# How M64 SID Wizard talks to Vessel

This documents the Vessel user-port protocol used by **M64 SID Wizard**
([github.com/M64GitHub/sid-wizard-vessel](https://github.com/M64GitHub/sid-wizard-vessel)),
a Vessel-specific fork of SID Wizard that uses Vessel's **NMI-on-MIDI-clock**
feature to sync playback to an external MIDI clock. It is the reference for the
`SidWizard*` integration tests in `vessel_test.cpp`.

Unlike Station64 (see `../station64/PROTOCOL.md`), SID Wizard is open source, so
this was read directly from the ACME assembly — no decrunching/emulation needed.
Source references below are to that repo.

## What makes it different from Station64

| | Station64 | SID Wizard |
|---|---|---|
| Status mask (`FD 06`) | not set | **`FF FF` (all realtime)** — needs clock/start/stop |
| Config flags (`FD 04`) | `01` (NMI) or `04` (transparent) | **`09` = NMI + NMI-status-only** |
| Core mechanism | reads buffered MIDI in main loop | **NMI fires on MIDI clock**, plays in the NMI handler |

## 1. Device open — full configuration

`MIDIC64.Open` / `OpenLgc` (`sources/MIDI-C64.asm`), run when the user selects
Vessel as the MIDI device:

```
; output mode: $DD00 bit2 = 1, DDRB = $FF
FD 00              ; Reset
FD 05 FF FF        ; Channel mask = all 16 channels
lda #$70           ; loop vEnCh, once per channel nibble 0..15:
  FD 07 ($70..$7F) ;   Control mask = all commands, every channel
FD 06 FF FF        ; Status mask = all real-time / channel-less messages  <-- key
```

So after open: `receiveChannelMask = 0xFFFF`, `receiveStatusMask = 0xFFFF`,
`receiveCommandMask[0..15] = 0x07`.

## 2. Enable NMI sync

`vessel_NMI_ON` (`sources/include/vessel/vesselnmi.inc`):

```
$DC0D <- $7F ; mask CIA1 timer IRQs
$DD0D <- $80 ; (ack/clear CIA2)
read $DC0D / $DD0D  ; clear pending
; output mode
FD 04 09           ; Config flags: bit0 NMI + bit3 NMI-status-only
install NMI vector -> VESSELNMI
$DD0D <- $90       ; enable /FLAG-driven NMI on the C64 side
```

`0x09` => `nmiEnabled = true`, `nmiStatusOnlyEnabled = true`, transparent off.

## 3. Behaviour SID Wizard relies on (verified against the firmware)

With the open + `FD 04 09` config:

- A MIDI **clock** (`$F8`), **start** (`$FA`) or **stop** (`$FC`) is **forwarded
  to the C64 and pulses /FLAG (an NMI)**. This is the sync tick.
- A **channel message** (e.g. note-on) is **not forwarded and raises no NMI**.
  In status-only mode the firmware's `NMI_CMD_WRAP` skips channel messages
  entirely, so only realtime status reaches the C64 — exactly what a clock-sync
  client wants, and it keeps the NMI rate low.

> Note: the README describes bit 3 as "enables NMI for status messages only",
> but the implementation also *suppresses forwarding* of channel messages while
> it is set. SID Wizard depends on this (it only consumes clock/start/stop), so
> the test pins the current behaviour.

## 4. NMI read handler

`VESSELNMI` -> `NMIReadMIDI` (`vesselnmi.inc`): switch to input mode, read the
count byte from `$DD01`, read that many bytes, watch for `StartSqPlay` (`$FA`)
and `StopSeqPlay` (`$FC`); the player advances one tick per `TimingClock`
(`$F8`). Then back to output mode.

## 5. Disable

`vessel_NMI_OFF`: `FD 04 00` (clear all flags) and restore the original NMI
vector.

## Firmware contract pinned by the tests

| SID Wizard step | Firmware requirement |
|---|---|
| device open | `receiveChannelMask == 0xFFFF`, `receiveStatusMask == 0xFFFF`, `receiveCommandMask[*] == 0x07` |
| `FD 04 09` | `nmiEnabled && nmiStatusOnlyEnabled && !transparent` |
| clock in | forwarded to `outBuf` **and** /FLAG pulsed |
| note-on in | **not** forwarded, /FLAG **not** pulsed |
| `FD 04 00` | flags cleared |
