# Vessel API

Following is example C64 assembly language for communicating with Vessel.
By default, Vessel will not generate any interrupts, and will not pass through
any received MIDI data to the C64, though it always passes through MIDI data
from the C64 no matter what.

## Configuration

Vessel needs to be configured (see [C64 to Vessel commands](#c64-to-vessel-commands))
to pass received MIDI data to the C64. Also, NMIs need to be explicitly enabled
if they are desired. Configuration is done by sending commands in output mode.

Following is an typical example to enable reception of all real time messages,
and note on and note off messages on channel 10, and enable NMIs.

```
        ;Set PA2 to 1 to signal OUTPUT
        lda $dd00
        ora #%00000100  ;Set bit2 to 1
        sta $dd00

        ;Set Port B to output
        lda #$ff
        sta $dd03

        ;Send command 0x4, enable NMI.
        lda #$fd
        sta $dd01
        lda #$04
        sta $dd01
        lda #$01
        sta $dd01

        ;Send command 0x6, enable all real time messages.
        lda #$fd
        sta $dd01
        lda #$06
        sta $dd01
        lda #$ff
        sta $dd01
        lda #$ff
        sta $dd01

        ;Send command 0x5, enable channel 10
        lda #$fd
        sta $dd01
        lda #$05
        sta $dd01
        lda #$02
        sta $dd01
        lda #$00
        sta $dd01

        ;Send command 0x7, enable note on and note off on channel 10
        lda #$fd
        sta $dd01
        lda #$07
        sta $dd01
        lda #$79 ;0111 - all messages - 1001 channel 10
        sta $dd01
```

## Outputting MIDI and commands

```
        ;Set PA2 to 1 to signal OUTPUT
        lda $dd00
        ora #%00000100  ;Set bit2 to 1
        sta $dd00

        ;Set Port B to output
        lda #$ff
        sta $dd03

        ;Bytes are sent by storing values to Port B
        lda #XX
        sta $dd01
```

## Inputting MIDI

```
        ;Reset PA2 to signal INPUT mode
        lda $dd00
        and #%11111011  ;Set bit2 to 0
        sta $dd00

        ;Set Port B to input
        lda #$00
        sta $dd03

        ;Read the available number of bytes. Max number of bytes in one go is 255 (not 256)
        ldy $dd01       ;Read bytecount from Port B
        beq .nobytesleft
.getmidiindata:
        lda $dd01 ;Read MIDI byte from Port B
        sta $XXXX ;Wherever desired
        dey
        bne .getmidiindata
.nobytesleft:
```

## C64 to Vessel commands

The C64 can send Vessel a command, by sending byte 0xFD (not used by MIDI),
and then a command, and then a fixed number of data bytes (depending on the
command - unless otherwise specified, a command is followed by 0 data bytes).

By default, transparent mode is off, NMI on external input is off, all MIDI channels
are masked and all status messages will be masked (no MIDI messages will be sent
to the C64) and MIDI through is disabled.

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
