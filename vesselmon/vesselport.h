// Vessel user-port access, abstracted behind four calls so the protocol logic
// in vesselmon_core.c can be unit tested on a host (test/vesselport_host.c)
// while the C64 build (vesselport_c64.c) drives the real CIA2 registers.
#ifndef VESSELMON_VESSELPORT_H
#define VESSELMON_VESSELPORT_H

#include <stdint.h>

void vp_output_mode(void); // C64 -> Vessel: PA2 = 1, port B output
void vp_input_mode(void);  // Vessel -> C64: PA2 = 0, port B input
uint8_t vp_read(void);     // read a byte from the port ($dd01)
void vp_write(uint8_t b);  // write a byte to the port ($dd01)

#endif // VESSELMON_VESSELPORT_H
