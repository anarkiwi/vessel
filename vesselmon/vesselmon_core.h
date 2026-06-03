// Vesselmon's Vessel I/O, independent of the C64 console so it can be tested on
// a host. See vesselmon_core.c.
#ifndef VESSELMON_CORE_H
#define VESSELMON_CORE_H

#include <stdint.h>

// Length of the version string Vessel returns to the Version command.
#define VESSEL_VERSION_LEN 8

// Read every byte Vessel currently has buffered into buf (up to 255 bytes);
// returns the count. Leaves the port in output mode. buf must hold >= 255
// bytes.
uint8_t vessel_read(uint8_t *buf);

// Send n bytes to Vessel (in output mode).
void vessel_send(const uint8_t *buf, uint8_t n);

// Probe for Vessel: send the Version command and read the reply into buf.
// Returns the reply length; VESSEL_VERSION_LEN means a Vessel was found and buf
// holds its version string. buf must hold >= 255 bytes.
uint8_t vessel_detect(uint8_t *buf);

#endif // VESSELMON_CORE_H
