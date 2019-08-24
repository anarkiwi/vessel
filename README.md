Vessel
======

![alt text](vessel.jpg)

Vessel is a high performance MIDI interface for the C64 based on an
Arduino Due-like platform, optimized to reduce interrupt load on the
C64.  Vessel is only compatible with software written specially for it
as it has its own protocol (see below).

Vessel's main points of differences from other C64 MIDI interfaces,
are that it uses the user port, not the cartridge port, and that it
does not use any CIA shift register ports which can accept only one
byte at a time (and can be used for other things).

If you are interested in acquiring a Vessel please contact josh@vandervecken.com.


Theory of operation
-------------------

The Vessel interface provides a buffered, 31250bps MIDI I/O
implementation via the C64's user port, without requiring the
C64 to handle an interupt.

The current implementation is effectively a virtual UART with 255 byte
transmit and receive buffers. Future versions of the driver will allow
the C64 to offload some MIDI parsing/processing to the driver (for
example, filter out unwanted MIDI messages).

The C64 provides a shift-register based TTL compatible serial port,
implemented on a CIA 6526/8520, via the user port. This is
inconvenient and slow as software must work with one bit at a time.

Fortunately the user port also provides an TTL 8 bit I/O port to the
same CIA chip, and the CIA can both generate a signal for external
hardware that the C64 has written a value to that port, and can
trigger an interrupt from external hardware that a value has been
written externally. There is no buffering.

The Vessel interface uses the 8 bit I/O port (PB0-7), /PC2, and
PA2. The C64 always controls the direction - whether it wants to read
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
while PA2 changes state input/output. We never interrupt the C64
so the FLAG lines are not used.


Upgrading firmware
------------------

The Arduino IDE can be used to reflash Vessel via the onboard USB-C port.
You should select the right port in the Tools menu before uploading.

NOTE: you will need to add the following to to WInterrupts.c:

    void PIOC_Handler (void) __attribute__ ((weak));

On Linux this can be located with the following command.

    josh@vek-x:~$ find .arduino15 -name WInterrupts.c
    .arduino15/packages/arduino/hardware/sam/1.6.12/cores/arduino/WInterrupts.c

If you will upgrade the IDE you will have to change this file again.


References
----------

C64 Programmers Reference Guide, Appendix M, 6526 Functional Description

https://archive.org/details/c64-programmer-ref/page/n449
