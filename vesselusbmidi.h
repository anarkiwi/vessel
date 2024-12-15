#include <USB-MIDI.h>

#define GETCIN(packet) (packet.header & 0x0f);
#define URXBUFFER_PUSHBACK1 fs.write(mPacket.byte1);
#define URXBUFFER_PUSHBACK2                                                    \
  {                                                                            \
    fs.write(mPacket.byte1);                                                   \
    fs.write(mPacket.byte2);                                                   \
  }
#define URXBUFFER_PUSHBACK3                                                    \
  {                                                                            \
    fs.write(mPacket.byte1);                                                   \
    fs.write(mPacket.byte2);                                                   \
    fs.write(mPacket.byte3);                                                   \
  }

midiEventPacket_t mPacket;

inline bool usbMidiRx() {
  bool packets = false;
  for (;;) {
    mPacket = MidiUSB.read();
    if (mPacket.header == 0) {
      break;
    }
    packets = true;
    flagPin.write(HIGH);
    byte cin = GETCIN(mPacket);
    byte len = usbMidi::cin2Len[cin][1];
    switch (len) {
    case 0:
      switch (cin) {
      case 0x4:
      case 0x7:
        URXBUFFER_PUSHBACK3
        break;
      case 0x5:
        URXBUFFER_PUSHBACK1
        break;
      case 0x6:
        URXBUFFER_PUSHBACK2
        break;
      }
      break;
    case 1:
      URXBUFFER_PUSHBACK1
      break;
    case 2:
      URXBUFFER_PUSHBACK2
      break;
    case 3:
      URXBUFFER_PUSHBACK3
      break;
    default:
      break;
    }
  }
  flagPin.write(LOW);
  return packets;
}
