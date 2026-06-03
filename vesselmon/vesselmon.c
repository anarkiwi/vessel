// Vesselmon - a small interactive C64 monitor for Vessel.
//
// At the "i/o?" prompt:
//   o : enter bytes to send (one per line); end with a blank line or a value
//       outside 0..255 (e.g. -1). The bytes are then transmitted to Vessel.
//   i : read and display the bytes Vessel currently has buffered.
//   anything else : quit.
//
// Example: send the Version command (253 3) then read back the 8-byte version
// string. This is a C rewrite of the original BASIC Vesselmon, built with the
// llvm-mos SDK.
#include "vesselmon_core.h"
#include "vesselport.h"
#include <stdint.h>
#include <stdio.h>

static uint8_t buf[256];

// Vessel's version string arrives as C64 screen codes; convert to PETSCII so it
// prints as text (letters are screen code + 64; digits/space already match).
static char screen_to_char(uint8_t sc) {
  if (sc >= 1 && sc <= 26) {
    return (char)(sc + 64);
  }
  return (char)sc;
}

// Probe for Vessel and report what was found.
static void detect_vessel(void) {
  uint8_t i, n = vessel_detect(buf);
  if (n == VESSEL_VERSION_LEN) {
    for (i = 0; i < n; i++) {
      putchar(screen_to_char(buf[i]));
    }
    putchar('\n');
  } else {
    printf("no vessel found\n");
  }
}

static void cmd_input(void) {
  uint8_t i, n = vessel_read(buf);
  printf("%u\n", (unsigned)n);
  for (i = 0; i < n; i++) {
    printf("%u %u\n", (unsigned)(i + 1), (unsigned)buf[i]);
  }
}

// Read one value from the user: 0..255, or -1 to finish (blank line, non-digit,
// or a value outside 0..255).
static int read_value(void) {
  char line[16];
  int v = 0, i = 0, sign = 1;
  if (fgets(line, sizeof line, stdin) == NULL) {
    return -1;
  }
  if (line[i] == '-') {
    sign = -1;
    i++;
  }
  if (line[i] < '0' || line[i] > '9') {
    return -1;
  }
  for (; line[i] >= '0' && line[i] <= '9'; i++) {
    v = v * 10 + (line[i] - '0');
  }
  v *= sign;
  if (v < 0 || v > 255) {
    return -1;
  }
  return v;
}

static void cmd_output(void) {
  uint8_t i, n = 0;
  int v;
  for (;;) {
    printf("%u ? ", (unsigned)(n + 1));
    v = read_value();
    if (v < 0) {
      break;
    }
    buf[n++] = (uint8_t)v;
    if (n == 255) {
      break;
    }
  }
  for (i = 0; i < n; i++) {
    printf("%u\n", (unsigned)buf[i]);
  }
  vessel_send(buf, n);
}

int main(void) {
  char line[16];
  vp_output_mode();
  detect_vessel();
  for (;;) {
    printf("i/o? ");
    if (fgets(line, sizeof line, stdin) == NULL) {
      break;
    }
    if (line[0] == 'i' || line[0] == 'I') {
      cmd_input();
    } else if (line[0] == 'o' || line[0] == 'O') {
      cmd_output();
    } else {
      break;
    }
  }
  return 0;
}
