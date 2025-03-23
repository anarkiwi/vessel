#include <USB-MIDI.h>

midiEventPacket_t mPacket;

#define GETCIN(packet) (packet.header & 0x0f);

#define PUSHBACK1(x)                                                           \
  { x(mPacket.byte1); }
#define PUSHBACK2(x)                                                           \
  {                                                                            \
    x(mPacket.byte1);                                                          \
    x(mPacket.byte2);                                                          \
  }
#define PUSHBACK3(x)                                                           \
  {                                                                            \
    x(mPacket.byte1);                                                          \
    x(mPacket.byte2);                                                          \
    x(mPacket.byte3);                                                          \
  }

#define USBMIDIRX(x, x1, x2, x3)                                               \
  inline bool x() {                                                            \
    bool packets = false;                                                      \
    for (;;) {                                                                 \
      mPacket = MidiUSB.read();                                                \
      if (mPacket.header == 0) {                                               \
        break;                                                                 \
      }                                                                        \
      packets = true;                                                          \
      byte cin = GETCIN(mPacket);                                              \
      byte len = usbMidi::cin2Len[cin][1];                                     \
      switch (len) {                                                           \
      case 0:                                                                  \
        switch (cin) {                                                         \
        case 0x4:                                                              \
        case 0x7:                                                              \
          x3;                                                                  \
          break;                                                               \
        case 0x5:                                                              \
          x1;                                                                  \
          break;                                                               \
        case 0x6:                                                              \
          x2;                                                                  \
          break;                                                               \
        default:                                                               \
          break;                                                               \
        }                                                                      \
        break;                                                                 \
      case 1:                                                                  \
        x1;                                                                    \
        break;                                                                 \
      case 2:                                                                  \
        x2;                                                                    \
        break;                                                                 \
      case 3:                                                                  \
        x3;                                                                    \
        break;                                                                 \
      default:                                                                 \
        break;                                                                 \
      }                                                                        \
    }                                                                          \
    return packets;                                                            \
  }

USBMIDIRX(transparentUsbMidiRx, PUSHBACK1(fs.ctrl_write), PUSHBACK2(fs.ctrl_write),
          PUSHBACK3(fs.ctrl_write));
USBMIDIRX(UsbMidiRx, PUSHBACK1(fs.qread), PUSHBACK2(fs.qread),
          PUSHBACK3(fs.qread));
