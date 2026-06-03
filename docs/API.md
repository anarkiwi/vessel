# Vessel API

The C64 talks to Vessel over the user port (CIA2, port B). The C64 always
controls the direction:

| Register | Meaning |
|----------|---------|
| `$dd00` bit 2 | data direction: `1` = C64 → Vessel (**output**, send MIDI/commands), `0` = Vessel → C64 (**input**, receive MIDI) |
| `$dd01` | the 8 data bits (PB0-7) |
| `$dd03` | port B data direction register (`$ff` = output, `$00` = input) |

To **send** MIDI: set output mode, then store each byte to `$dd01`.

To **receive** MIDI: set input mode, read the byte count from `$dd01`, then read
that many bytes. Vessel masks all MIDI by default, so a program must first
configure it (see the commands below) or enable transparent mode — otherwise no
MIDI is forwarded to the C64. The maximum number of bytes returned in one read
is 255.

By default: transparent mode is off, NMI on external input is off, all MIDI
channels are masked, all status messages are masked (no MIDI is sent to the
C64), and MIDI through is disabled.

## C64 to Vessel commands

The C64 sends Vessel a command by writing byte `$fd` (never a real MIDI byte),
then the command number, then a fixed number of data bytes (0 unless noted).

|byte|command     |arg bytes|description
|----|------------|---------|--------------------------------------------------------------------------
|0x00|Reset       |         |Vessel will reset to default config.
|0x01|Purge       |         |Vessel will discard any buffered data.
|0x02|Panic       |         |Reserved for future implementation. Send all notes off on all channels.
|0x03|Version     |         |Vessel will return a version string (currently C64 screen code "vessel00").
|0x04|Config      |CF       |CF bit 0 enables NMI, bit 1 enables MIDI through, bit 2 enables transparent (no MIDI parsing) mode, bit 3 enables NMI for status messages only (requires bit 0).
|0x05|Channel mask|HH LL    |High byte (HH), low byte (LL) for channels 1 to 16.
|0x06|Status mask |HH LL    |High byte (HH), low byte (LL) for channel-less messages F0 to FF.
|0x07|Control mask|CM       |MSB unused, 3 high bits command mask, 4 low bits channel.

## Working examples

Complete, runnable C64 programs that assemble with ACME live in
[`../examples/`](../examples) (and are assembled in CI):

| Example | Shows |
|---------|-------|
| [`send.asm`](../examples/send.asm) | outputting MIDI |
| [`receive.asm`](../examples/receive.asm) | configuration + inputting MIDI |
| [`echo.asm`](../examples/echo.asm) | transparent-mode passthrough/loopback |
| [`vessel.inc`](../examples/vessel.inc) | the shared output/input/read routines |

For NMI-on-MIDI-clock sync and how real applications drive Vessel, see
[`../test/sidwizard/PROTOCOL.md`](../test/sidwizard/PROTOCOL.md) and
[`../test/station64/PROTOCOL.md`](../test/station64/PROTOCOL.md).
