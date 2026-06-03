// Minimal host mock of the Arduino USB-MIDI library.
//
// vesselusbmidi.h consumes USB-MIDI event packets from the global MidiUSB
// object and uses the usbMidi::cin2Len table to decode them. On the host there
// is no USB stack, so MidiUSB.read() draws from an injectable queue that tests
// populate. The cin2Len table is copied verbatim from the real library
// (lathoub/Arduino-USBMIDI, src/USB-MIDI_defs.h) so decoding behaviour matches.

#ifndef VESSEL_HOST_USB_MIDI_H
#define VESSEL_HOST_USB_MIDI_H

#include <cstdint>
#include <deque>

#include "Arduino.h"

// Layout matches the Arduino MIDIUSB midiEventPacket_t.
struct midiEventPacket_t {
  uint8_t header;
  uint8_t byte1;
  uint8_t byte2;
  uint8_t byte3;
};

namespace usbMidi {
// {CIN, number of MIDI data bytes}. CINs 4-7 (SysEx) report 0 here and are
// special-cased by vesselusbmidi.h.
static byte cin2Len[][2] = {{0, 0},  {1, 0},  {2, 2},  {3, 3}, {4, 0},  {5, 0},
                            {6, 0},  {7, 0},  {8, 3},  {9, 3}, {10, 3}, {11, 3},
                            {12, 2}, {13, 2}, {14, 3}, {15, 1}};
} // namespace usbMidi

class HostMidiUSB {
public:
  // Tests push packets here; the firmware drains them via read().
  std::deque<midiEventPacket_t> rx;

  midiEventPacket_t read() {
    if (rx.empty()) {
      return midiEventPacket_t{0, 0, 0, 0};
    }
    midiEventPacket_t p = rx.front();
    rx.pop_front();
    return p;
  }
};

extern HostMidiUSB MidiUSB;

#endif // VESSEL_HOST_USB_MIDI_H
