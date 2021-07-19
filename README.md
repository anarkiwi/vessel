Vessel
======

![alt text](vessel.jpg)

Vessel is a high performance MIDI interface for the C64 based on an
Arduino Due-like platform, optimized to reduce interrupt load on the
C64.  Vessel is only compatible with software written specially for it
as it has its own protocol.

There is a reference application, [Vessel MIDI Player](https://github.com/anarkiwi/vmp/releases),
and ports of SID Wizard - (a basic one, [SID Wizard 1.8.7](https://github.com/anarkiwi/sid-wizard/releases), and an advanced one [M64 SID Wizard](https://github.com/M64GitHub/sid-wizard/tree/m64/add-external-midi-sync) that supports variable external clock),
[Vicficken](https://github.com/anarkiwi/vvf/releases), as well as
[Vessel ASID Player](https://github.com/anarkiwi/vap/releases) which supports ASID.

Vessel's main points of differences from other C64 MIDI interfaces are:

* it uses the user port, not the cartridge port
* it does not use/need any CIA shift register ports
* it can transfer multiple bytes per transaction (unlike ACIA/6850 designs)
* the C64 can have configure Vessel to filter MIDI messages to save CPU time

If you are interested in acquiring a Vessel please contact josh@vandervecken.com.


Theory of operation
-------------------

The Vessel interface provides a buffered, 31250bps MIDI I/O
implementation via the C64's user port, without requiring the
C64 to handle an interupt (though the C64 can choose to receive
one - see below).

The C64 provides a shift-register based TTL compatible serial port,
implemented on a CIA 6526/8520, via the user port. This is
inconvenient and slow as software must work with one bit at a time.

Fortunately the user port also provides an (unbuffered) TTL 8 bit I/O
interface to the same CIA chip. The CIA can both generate a signal for
external hardware that the C64 has written a value to that port, and
allows the external hardware to trigger an NMI (Vessel uses this to tell
the C64 there is at least one byte read to read). 

The Vessel interface uses this 8 bit I/O port (PB0-7), /PC2, PA2, and /FLAG. 
The C64 always controls the direction - whether it wants to read
from Vessel or write. Vessel will buffer incoming MIDI data until the
C64 wants to read the buffer, and will also write out from the buffer
any data that the C64 has written. The C64 can read and write to
Vessel faster than MIDI's datarate, so the C64 must not exceed MIDI
bandwidth.

When PA2 is high, the C64 wishes to transmit. /PC2 will go low for one
C64 cycle, after the C64 has written a byte. We catch the rising edge
of that low pulse and read PB0-7, and buffer the byte to be transmitted
over the MIDI port.

When PA2 is low, the C64 wishes to receive. Vessel writes the number
of bytes it will transmit to the C64 (queued until now from the MIDI
port) to PB0-7.  Then for each /PC2 low pulse (signifying the C64 has
read the port), we will send another byte (if we have any buffered,
from the MIDI port).

The C64 has no way to know that Vessel has successfully read or
written a byte, we simply have to keep up with its schedule. An
Arduino Uno isn't fast enough, but a Due is (with time for other tasks
while PA2 changes state input/output).

Vessel API
----------

Following is example C64 assembly language for communicating with Vessel.
By default, Vessel will not generate any interrupts, and will not pass through
any received MIDI data to the C64, though it always passes through MIDI data
from the C64 no matter what.


Configuration
-------------

Vessel needs to be configured (see C64 to Vessel commands) to pass received
MIDI data to the C64. Also, NMIs need to be explicitly enabled if they are desired.
Configuration is done by sending commands in output mode.

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


Outputting MIDI and commands
----------------------------

```
        ;Set PA2 to 1 to signal OUTPUT
	lda $dd00
	ora #%00000100	;Set bit2 to 1
	sta $dd00

	;Set Port B to output
	lda #$ff
	sta $dd03

        ;Bytes are sent by storing values to Port B
        lda #XX
	sta $dd01
```

Inputting MIDI
--------------

```
	;Reset PA2 to signal INPUT mode
	lda $dd00
	and #%11111011	;Set bit2 to 0
	sta $dd00

	;Set Port B to input
	lda #$00
	sta $dd03

	;Read the available number of bytes. Max number of bytes in one go is 255 (not 256)
	ldy $dd01	;Read bytecount from Port B
	beq .nobytesleft
.getmidiindata:
	lda $dd01 ;Read MIDI byte from Port B
	sta $XXXX ;Wherever desired
	dey
	bne .getmidiindata
.nobytesleft:
```

C64 to Vessel commands
----------------------

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


Upgrading firmware
------------------

The Arduino IDE can be used to reflash Vessel via the onboard USB-C port.
You should select the right port in the Tools menu before uploading
(Vessel will show as an Arduino Due, native port). You will also need
the "MIDI Library" (by Francois Best) installed, via Tools/Manage Libraries,
and support for SAM boards installed (Tools/Board Manager).

NOTE: you will need to add the following to to WInterrupts.c:

    void PIOC_Handler (void) __attribute__ ((weak));

On Linux, WInterrupts.c can be located with:

    $ find .arduino15/ -name WInterrupts.c | grep sam
    .arduino15/packages/arduino/hardware/sam/1.6.12/cores/arduino/WInterrupts.c

You will also need to change Arduino's default optimization settings. Change
all references to -Os, to -O2, in platforms.txt.

On Linux, platforms.txt can be located with:

    $ find .arduino15/ -name platform.txt | grep sam
    .arduino15/packages/arduino/hardware/sam/1.6.12/platform.txt

If you will upgrade the IDE you will have to make these changes again. 


Vesselmon
---------

Vesselmon is a small C64 test program for Vessel, which allows sending and receiving
(including with a loopback MIDI cable).

In this example, the user sends the version command and receives the version string response.

```
I/O? O
 1 ? 253
 2 ? 3
 3 ? -1
 253
 3
I/O? I
 8
 1         22
 2         5
 3         19
 4         19
 5         5
 6         12
 7         48
 8         48
I/O? X
```

References
----------

C64 Programmers Reference Guide, Appendix M, 6526 Functional Description

https://archive.org/details/c64-programmer-ref/page/n449
