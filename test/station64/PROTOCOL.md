# How Station64 talks to Vessel

This documents the Vessel user-port protocol as actually used by the
**Station64 V2.6** application (DJ Indikator, 2021,
[CSDb #207214](https://csdb.dk/release/?id=207214)). It is the reference for the
`Station64*` integration tests in `vessel_test.cpp`, which replay these exact
byte sequences against the host build of the firmware so we can detect any
future change that would break Station64.

## How this was obtained

`STA64` on the disk is a crunched `$0801` autostart (Shannon entropy ~7.9
bits/byte), so the driver does not exist in the packed file — it only appears in
RAM after the decruncher runs. The steps:

1. Pull `sta64.d64` from CSDb and extract `STA64` (a small D64 reader; the file
   loads at `$0801`, 23798 bytes).
2. Boot it under `asid-vice` (VICE + binary monitor) using the
   `/scratch/anarkiwi/vice-driver` automation, let it decrunch to its splash
   screen ("MIDI-SID PRODUCTION SYSTEM"), then dump resident RAM `$0800-$CFFF`
   via the binary monitor's `mem_get`.
3. Scan the dump for CIA2 user-port accesses (`$DD00/$DD01/$DD03/$DD0D`) and the
   Vessel command marker `#$FD`. They cluster in one driver at **`$6700-$68E0`**.
4. Disassemble that region with `da65`.

## CIA2 user-port lines (per README "Theory of operation")

| Reg | Role |
|-----|------|
| `$DD00` | PA2 (bit 2) selects direction: **1 = output** (C64→Vessel), **0 = input** (Vessel→C64) |
| `$DD01` | Port B, the 8 data bits PB0-7 |
| `$DD03` | Port B DDR: `$FF` = output, `$00` = input |
| `$DD0D` | CIA2 ICR — enable/ack the /FLAG-driven NMI |

Mode helpers in the driver:

```
output mode ($67F9):  lda $DD00 / ora #$04 / sta $DD00   ; PA2 = 1
                      lda #$FF  / sta $DD03              ; port B = output
input mode  ($67ED):  lda $DD00 / and #$FB / sta $DD00   ; PA2 = 0
                      lda #$00  / sta $DD03              ; port B = input
```

## 1. Vessel detection — Version command (`$6809`)

```
jsr output_mode
lda #$FD / sta $DD01      ; command marker
lda #$03 / sta $DD01      ; command 3 = Version
jsr input_mode
ldy $DD01                 ; read COUNT byte Vessel offers
cpy #$08                  ; REQUIRE exactly 8  <-- Vessel-present test
bne not_present
loop: lda $DD01 / dey / bne loop   ; read the 8 version bytes
jsr output_mode
```

**Contract:** after `FD 03`, the firmware must have **8** bytes pending
(`vesselConfig.pendingOut == 8`, sent as the leading count byte by
`outputMode()`), and those 8 bytes are the version string. If the count is not
8, Station64 decides Vessel is absent.

## 2. Configuration

Station64 has two init variants. Both first Reset, set masks, then set flags.

`L6884` (set masks), shared by both:

```
lda #$FD / sta $DD01 / lda #$05 / sta $DD01   ; command 5 = Channel mask
lda #$FF / sta $DD01 / sta $DD01              ; HH=$FF, LL=$FF  -> all 16 channels
lda #$70
loop ($6898):
  ldx #$FD / stx $DD01 / ldx #$07 / stx $DD01 ; command 7 = Control/command mask
  sta $DD01                                   ; data = $70..$7F
  adc #$01 / cmp #$80 / bne loop              ; once per channel nibble 0..15
```

`$70..$7F` => high nibble `7` (command bits = note-on|note-off|CC), low nibble =
channel `0..15`. So: **all 16 channels, note-on/off + CC enabled on each.**

**NMI-sync variant (`$6852`):**

```
FD 00            ; Reset
<L6884 masks>
FD 04 01         ; Config flags = $01 (bit0 = NMI enabled)
$DD0D <- $90     ; CIA2 ICR: enable /FLAG NMI on the C64 side
```

**Transparent variant (`$682A`):**

```
FD 00            ; Reset
<L6884 masks>
FD 04 04         ; Config flags = $04 (bit2 = transparent / no parsing)
$DD0D <- $7F     ; CIA2 ICR: disable CIA NMIs
```

## 3. Receiving MIDI (`$68AC`)

```
input_mode
ldx $DD01                 ; COUNT byte
beq done
loop: lda $DD01 / sta $0200,y / iny / <advance ptr> / dex / bne loop
done: output_mode
```

Matches the firmware's `outputMode()`: it writes `pendingOut` as the leading
count byte, then that many buffered MIDI bytes.

## Summary of the firmware contract Station64 depends on

| Station64 action | Firmware requirement (tested on host) |
|------------------|----------------------------------------|
| `FD 03` probe | `pendingOut == 8`, bytes == `versionStr` |
| `FD 05 FF FF` | `receiveChannelMask == 0xFFFF` |
| 16× `FD 07 7n` | `receiveCommandMask[n] == 0x07` for n=0..15 |
| `FD 04 01` | `nmiEnabled == true`, `transparent == false` |
| `FD 04 04` | `transparent == true` |
| `FD 00` | full reset of the above |
| filtered MIDI in | forwarded bytes appear in `outBuf`, count in `pendingOut` |
