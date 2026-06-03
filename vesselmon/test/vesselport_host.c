// A simulated Vessel port for host unit tests. It mimics how Vessel presents
// received data: in input mode the first read returns the byte count, then each
// read returns the next queued byte.
#include "vesselport_host.h"
#include "../vesselport.h"
#include <string.h>

uint8_t host_tx[1024];
unsigned host_tx_len;
int host_mode_is_output;

static uint8_t rx_queue[256];
static uint8_t rx_len;
static uint8_t rx_pos;
static int count_returned;

void host_reset(void) {
  host_tx_len = 0;
  host_mode_is_output = 1;
  rx_len = 0;
  rx_pos = 0;
  count_returned = 0;
}

void host_set_rx(const uint8_t *bytes, uint8_t n) {
  memcpy(rx_queue, bytes, n);
  rx_len = n;
  rx_pos = 0;
  count_returned = 0;
}

void vp_output_mode(void) { host_mode_is_output = 1; }

void vp_input_mode(void) {
  host_mode_is_output = 0;
  rx_pos = 0;
  count_returned = 0;
}

uint8_t vp_read(void) {
  if (!count_returned) {
    count_returned = 1;
    return rx_len; // first read in input mode is the count
  }
  if (rx_pos < rx_len) {
    return rx_queue[rx_pos++];
  }
  return 0;
}

void vp_write(uint8_t b) {
  if (host_tx_len < sizeof(host_tx)) {
    host_tx[host_tx_len++] = b;
  }
}
