// C64 user-port access for Vesselmon, via the CIA2 registers:
//   $dd00 bit 2 : data direction (1 = C64 -> Vessel, 0 = Vessel -> C64)
//   $dd01       : the 8 data bits (PB0-7)
//   $dd03       : port B data direction register ($ff = output, $00 = input)
#include "vesselport.h"

#define CIA2_PRA (*(volatile uint8_t *)0xdd00)
#define CIA2_PRB (*(volatile uint8_t *)0xdd01)
#define CIA2_DDRB (*(volatile uint8_t *)0xdd03)

void vp_output_mode(void) {
  CIA2_PRA |= 0x04;
  CIA2_DDRB = 0xff;
}

void vp_input_mode(void) {
  CIA2_PRA &= (uint8_t)~0x04;
  CIA2_DDRB = 0x00;
}

uint8_t vp_read(void) { return CIA2_PRB; }

void vp_write(uint8_t b) { CIA2_PRB = b; }
