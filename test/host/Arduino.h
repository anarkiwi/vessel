// Minimal host mock of the Arduino core API.
//
// This is *not* a faithful Arduino emulation; it provides just enough of the
// surface used by vessel.ino, CRDigitalPin.h, platform.h (ARCH_HOST branch),
// and the vendored MIDI library to compile and run the firmware logic on a PC.
// Hardware side effects (pin levels) are routed through host_sim so tests can
// observe them.

#ifndef VESSEL_HOST_ARDUINO_H
#define VESSEL_HOST_ARDUINO_H

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "host_sim.h"

typedef uint8_t byte;
typedef bool boolean;

// Pin modes / levels.
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Interrupt edge modes.
#define CHANGE 1
#define FALLING 2
#define RISING 3

// The firmware brackets register pokes with noInterrupts()/interrupts(); on the
// host there is nothing to disable.
inline void noInterrupts() {}
inline void interrupts() {}

inline void pinMode(uint8_t pin, uint8_t mode) {
  (void)pin;
  (void)mode;
}

inline void digitalWrite(uint8_t pin, uint8_t value) {
  hostsim::pinLevel[pin] = value;
}

inline int digitalRead(uint8_t pin) { return hostsim::pinLevel[pin]; }

// Interrupts never actually fire on the host; tests call the firmware's ISR
// handlers directly. These stubs let setup() link.
inline uint8_t digitalPinToInterrupt(uint8_t pin) { return pin; }
inline void attachInterrupt(uint8_t, void (*)(), int) {}
inline void detachInterrupt(uint8_t) {}

inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}

#endif // VESSEL_HOST_ARDUINO_H
