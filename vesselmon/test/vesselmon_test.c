// Host unit tests for Vesselmon's Vessel I/O core. Compiles with a normal C
// compiler against the simulated port; no C64 or emulator needed.
#include "../vesselmon_core.h"
#include "vesselport_host.h"
#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
      failures++;                                                              \
    }                                                                          \
  } while (0)

// A read returns the count byte first, then that many bytes, and leaves the
// port in output mode (matching Vessel's protocol).
static void test_read_returns_count_then_bytes(void) {
  static const uint8_t version[] = {22, 5, 19, 19, 5, 12, 48, 48}; // "VESSEL00"
  uint8_t buf[256];
  uint8_t n;
  host_reset();
  host_set_rx(version, sizeof(version));
  n = vessel_read(buf);
  CHECK(n == sizeof(version));
  CHECK(memcmp(buf, version, sizeof(version)) == 0);
  CHECK(host_mode_is_output == 1);
}

// An empty Vessel buffer yields a zero count.
static void test_read_empty(void) {
  uint8_t buf[256];
  host_reset();
  CHECK(vessel_read(buf) == 0);
}

// Sending writes the bytes verbatim, in output mode.
static void test_send_writes_bytes_in_output_mode(void) {
  static const uint8_t version_cmd[] = {0xfd, 0x03}; // marker, Version command
  host_reset();
  vessel_send(version_cmd, sizeof(version_cmd));
  CHECK(host_mode_is_output == 1);
  CHECK(host_tx_len == sizeof(version_cmd));
  CHECK(host_tx[0] == 0xfd);
  CHECK(host_tx[1] == 0x03);
}

// Detection sends the Version command and reports the 8-byte version reply.
static void test_detect_finds_vessel(void) {
  static const uint8_t version[] = {22, 5, 19, 19, 5, 12, 48, 48};
  uint8_t buf[256];
  uint8_t n;
  host_reset();
  host_set_rx(version, sizeof(version));
  n = vessel_detect(buf);
  CHECK(n == VESSEL_VERSION_LEN);
  CHECK(memcmp(buf, version, sizeof(version)) == 0);
  CHECK(host_tx_len == 2); // the Version command was sent
  CHECK(host_tx[0] == 0xfd);
  CHECK(host_tx[1] == 0x03);
}

// With no Vessel responding, the reply is not the version length.
static void test_detect_no_vessel(void) {
  uint8_t buf[256];
  host_reset(); // empty rx -> count 0
  CHECK(vessel_detect(buf) != VESSEL_VERSION_LEN);
}

int main(void) {
  test_read_returns_count_then_bytes();
  test_read_empty();
  test_send_writes_bytes_in_output_mode();
  test_detect_finds_vessel();
  test_detect_no_vessel();
  if (failures) {
    printf("%d check(s) FAILED\n", failures);
    return 1;
  }
  printf("all vesselmon tests passed\n");
  return 0;
}
