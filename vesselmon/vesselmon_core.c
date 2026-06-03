#include "vesselmon_core.h"
#include "vesselport.h"

uint8_t vessel_read(uint8_t *buf) {
  uint8_t i, n;
  vp_input_mode();
  n = vp_read(); // first byte is the count Vessel will send
  for (i = 0; i < n; i++) {
    buf[i] = vp_read();
  }
  vp_output_mode();
  return n;
}

void vessel_send(const uint8_t *buf, uint8_t n) {
  uint8_t i;
  vp_output_mode();
  for (i = 0; i < n; i++) {
    vp_write(buf[i]);
  }
}

uint8_t vessel_detect(uint8_t *buf) {
  static const uint8_t version_cmd[2] = {0xfd, 0x03}; // marker, Version command
  vessel_send(version_cmd, 2);
  return vessel_read(buf);
}
