# Tests

## Host tests

Vessel's parsing logic (the C64 command protocol, MIDI message filtering, and
USB-MIDI packet decoding) can be unit tested on a regular PC, with no Arduino
hardware. The tests `#include` the real `vessel.ino` and compile it natively:
the hardware seams in `platform.h`, `pins.h`, and `CRDigitalPin.h` have an
`ARCH_HOST` branch (selected only for the host build) that routes the C64 user
port and MIDI UART through in-memory buffers, so a test can feed bytes in and
assert on what the firmware parses and emits. The real "MIDI Library" is used,
so the MIDI parser is exercised end to end.

The tests, mocks, and build live under `test/host/`.

### Application integration tests

`vessel_test.cpp` also replays the exact user-port command sequences real
Vessel applications send, and asserts the firmware still honours each one. Two
applications, driving Vessel in deliberately different ways, are covered:

* **`Station64*`** — version probe, both init variants (transparent and NMI),
  channel/command masks, and a MIDI receive round-trip. Protocol recovered by
  disassembling the (crunched) application in an emulator; see
  [`../test/station64/PROTOCOL.md`](../test/station64/PROTOCOL.md).
* **`SidWizard*`** — [M64 SID Wizard](https://github.com/M64GitHub/sid-wizard-vessel),
  which uses Vessel's NMI-on-MIDI-clock sync (status mask + config flag `$09`,
  NMI-status-only). Covers that a MIDI clock is forwarded *and* pulses /FLAG
  while channel messages are suppressed. Protocol read from its open ACME
  source; see [`../test/sidwizard/PROTOCOL.md`](../test/sidwizard/PROTOCOL.md).
  The polled [anarkiwi/sid-wizard](https://github.com/anarkiwi/sid-wizard) fork
  (SID Wizard 1.97) carries a mirror of these tests on the SID-Wizard side,
  asserting the bytes it emits with a 6502 simulator.

These run in the same suite with no extra dependencies, so a firmware change
that breaks either application fails CI. (A heavier optional lane could
co-simulate an application live in `asid-vice` against this host build; the
captured byte sequences keep the fast path dependency-free.)

Run them in Docker (matches CI exactly):

```
docker build -t vessel-test .
```

The image build configures, compiles, and runs the suite, so it fails if any
test fails. To re-run without rebuilding: `docker run --rm vessel-test`.

Run them directly (needs `cmake`, a C++ compiler, `git`, and `clang-format`):

```
cmake -S test/host -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

GoogleTest and the MIDI Library are fetched automatically (pinned versions) by
CMake.

## Code formatting

All C/C++ sources are formatted with `clang-format` (LLVM style; see
`.clang-format`). The `clang_format` ctest case fails if any tracked source is
not formatted, so CI catches drift. To reformat in place:

```
test/host/tools/check-format.sh --fix
```
