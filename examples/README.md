# Vessel examples

Small, complete C64 programs showing how to talk to Vessel over the user port.
Each is a runnable autostart `.prg` (`LOAD"NAME.PRG",8,1` then `RUN`). See
[../docs/API.md](../docs/API.md) for the command set and
[vessel.inc](vessel.inc) for the shared user-port routines.

| Example | Shows | What it does |
|---|---|---|
| [send.asm](send.asm) | outputting MIDI | plays middle C on channel 1 (note on/off) repeatedly |
| [receive.asm](receive.asm) | configuration + inputting MIDI | configures Vessel to forward note/control + real-time on all channels, then displays received bytes |
| [echo.asm](echo.asm) | transparent mode | enables raw passthrough and echoes received MIDI back out |

## Building

Requires [ACME](https://sourceforge.net/projects/acme-crossass/) (Debian/Ubuntu:
`apt-get install acme`):

```
make            # assemble all examples to .prg
make clean
```

CI assembles these on every push (`.github/workflows/examples.yml`).

## Compatibility

The configuration sequences here are exercised against the real firmware by the
host tests (`EchoExample*` / `ReceiveExample*` in `../test/host/vessel_test.cpp`),
so they stay in sync with Vessel. Note that current firmware masks all MIDI by
default: a program must enable transparent mode or set the channel/status/command
masks before it will receive anything (older examples that skipped this no longer
work as-is).
