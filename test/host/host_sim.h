// Host-test simulation seam for Vessel.
//
// On the embedded target, Vessel talks to the C64 user port and the MIDI DIN
// UART through hardware registers (see platform.h). For host tests we replace
// those registers with in-memory FIFOs/buffers so the *real* firmware code in
// vessel.ino can run on a PC. A test pushes bytes the C64 "wrote" into portIn,
// drives the firmware, then inspects what it produced.

#ifndef VESSEL_HOST_SIM_H
#define VESSEL_HOST_SIM_H

#include <cstdint>
#include <deque>
#include <vector>

namespace hostsim {

// Bytes the C64 has written to the user port. getByte() pops the front.
extern std::deque<uint8_t> portIn;
// Bytes the firmware wrote to the user port via setByte() (to the C64).
extern std::vector<uint8_t> portOut;
// Incoming MIDI DIN bytes; uartRxready()/uartRead() consume these.
extern std::deque<uint8_t> uartIn;
// MIDI DIN bytes the firmware transmitted via uartWrite().
extern std::vector<uint8_t> uartTx;
// Simulated digital pin levels, indexed by Arduino pin number.
extern bool pinLevel[256];
// Count of writes to each pin, so tests can detect NMI pulses (a HIGH/LOW pair
// on the FLAG pin) rather than just final level.
extern unsigned long pinWrites[256];

// Clear all simulated I/O state. Call between tests.
void reset();

} // namespace hostsim

#endif // VESSEL_HOST_SIM_H
