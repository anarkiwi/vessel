# Vesselmon

Vesselmon is a small interactive C64 monitor for Vessel: it can send arbitrary
bytes to Vessel and read bytes back (e.g. with a loopback MIDI cable). It is
written in C and built with the [llvm-mos SDK](https://github.com/llvm-mos/llvm-mos-sdk);
the sources and build live in [`../vesselmon/`](../vesselmon), with checked-in
static builds [`vesselmon.prg`](../vesselmon/vesselmon.prg) and
[`vesselmon.d64`](../vesselmon/vesselmon.d64).

On start-up it probes for Vessel (sends the Version command and checks the
reply): it prints the version string if a Vessel is found, otherwise
`no vessel found`. Then at the `i/o?` prompt:

* **`o`** — enter bytes to send, one per line; end with a blank line or a value
  outside 0..255 (e.g. `-1`). The bytes are transmitted to Vessel.
* **`i`** — read and display the bytes Vessel currently has buffered.
* anything else — quit.

This example sends the Version command (`253 3`) then reads the 8-byte version
string back:

```
vessel00
i/o? o
 1 ? 253
 2 ? 3
 3 ? -1
 253
 3
i/o? i
 8
 1 22
 2 5
 3 19
 4 19
 5 5
 6 12
 7 48
 8 48
i/o? x
```

## Building and testing

```
cd vesselmon
make            # build vesselmon.prg and vesselmon.d64 (needs llvm-mos + c1541)
make test       # build and run the host unit tests
make format-check
```

The Vessel I/O logic is split into a portable core (`vesselmon_core.c`) behind a
small port interface (`vesselport.h`), so it is unit tested on a host against a
simulated port (`test/`) with no emulator. CI builds the program with llvm-mos,
runs the host tests, and enforces clang-format LLVM style
(`.github/workflows/vesselmon.yml`).
