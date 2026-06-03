// Test control surface for the simulated Vessel port (vesselport_host.c).
#ifndef VESSELMON_VESSELPORT_HOST_H
#define VESSELMON_VESSELPORT_HOST_H

#include <stdint.h>

// Bytes the C64 (vessel_send) wrote to the port, in order.
extern uint8_t host_tx[1024];
extern unsigned host_tx_len;
// Non-zero when the port is in output mode (C64 -> Vessel).
extern int host_mode_is_output;

// Reset all simulated state.
void host_reset(void);
// Queue the bytes Vessel will deliver to the C64 on the next read.
void host_set_rx(const uint8_t *bytes, uint8_t n);

#endif // VESSELMON_VESSELPORT_HOST_H
