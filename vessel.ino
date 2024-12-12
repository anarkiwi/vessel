// Vessel

// Copyright 2019 Josh Bailey (josh@vandervecken.com)

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "USB-MIDI.h"
#include <MIDI.h>

enum MidiType: uint8_t
{
    InvalidType           = 0x00,    ///< For notifying errors
    NoteOff               = 0x80,    ///< Channel Message - Note Off
    NoteOn                = 0x90,    ///< Channel Message - Note On
    AfterTouchPoly        = 0xA0,    ///< Channel Message - Polyphonic AfterTouch
    ControlChange         = 0xB0,    ///< Channel Message - Control Change / Channel Mode
    ProgramChange         = 0xC0,    ///< Channel Message - Program Change
    AfterTouchChannel     = 0xD0,    ///< Channel Message - Channel (monophonic) AfterTouch
    PitchBend             = 0xE0,    ///< Channel Message - Pitch Bend
    SystemExclusive       = 0xF0,    ///< System Exclusive
	SystemExclusiveStart  = SystemExclusive,   ///< System Exclusive Start
    TimeCodeQuarterFrame  = 0xF1,    ///< System Common - MIDI Time Code Quarter Frame
    SongPosition          = 0xF2,    ///< System Common - Song Position Pointer
    SongSelect            = 0xF3,    ///< System Common - Song Select
    Undefined_F4          = 0xF4,
    Undefined_F5          = 0xF5,
    TuneRequest           = 0xF6,    ///< System Common - Tune Request
	SystemExclusiveEnd    = 0xF7,    ///< System Exclusive End
    Clock                 = 0xF8,    ///< System Real Time - Timing Clock
    Undefined_F9          = 0xF9,
    Tick                  = Undefined_F9, ///< System Real Time - Timing Tick (1 tick = 10 milliseconds)
    Start                 = 0xFA,    ///< System Real Time - Start
    Continue              = 0xFB,    ///< System Real Time - Continue
    Stop                  = 0xFC,    ///< System Real Time - Stop
    Undefined_FD          = 0xFD,
    ActiveSensing         = 0xFE,    ///< System Real Time - Active Sensing
    SystemReset           = 0xFF,    ///< System Real Time - System Reset
};

const uint8_t type2cin[][2] = { {MidiType::InvalidType,0}, {MidiType::NoteOff,8}, {MidiType::NoteOn,9}, {MidiType::AfterTouchPoly,0xA}, {MidiType::ControlChange,0xB}, {MidiType::ProgramChange,0xC}, {MidiType::AfterTouchChannel,0xD}, {MidiType::PitchBend,0xE} };
const uint8_t system2cin[][2] = { {MidiType::SystemExclusive,0}, {MidiType::TimeCodeQuarterFrame,2}, {MidiType::SongPosition,3}, {MidiType::SongSelect,2}, {0,0}, {0,0}, {MidiType::TuneRequest,5}, {MidiType::SystemExclusiveEnd,0}, {MidiType::Clock,0xF}, {0,0}, {MidiType::Start,0xF}, {MidiType::Continue,0xF}, {MidiType::Stop,0xF}, {0,0}, {MidiType::ActiveSensing,0xF}, {MidiType::SystemReset,0xF} };
const byte cin2Len[][2] = { {0,0}, {1,0}, {2,2}, {3,3}, {4,0}, {5,0}, {6,0}, {7,0}, {8,3}, {9,3}, {10,3}, {11,3}, {12,2}, {13,2}, {14,3}, {15,1} };

#define GETCIN(packet) (packet.header & 0x0f);

#include <Arduino.h>
#include "pins.h"
#include "CRDigitalPin.h"
#include "platform.h"

const char versionStr[] = {
  0x16, // V
  0x05, // E
  0x13, // S
  0x13, // S
  0x05, // E
  0x0C, // L
  0x30, // 0
  0x30, // 0
};

// The C64 can send us a command by sending a reserved cmd byte, then another byte specifying the command number, then any other bytes the command requires.
// Each command has a fixed number of bytes expected after the command byte.
const byte vesselCmd = 0xfd;

const byte c64Pins[] = {C64_PB0, C64_PB1, C64_PB2, C64_PB3, C64_PB4, C64_PB5, C64_PB6, C64_PB7};
const uint16_t OUT_BUF_SIZE = 256; // Ring buffer for bytes to send to C64
const uint16_t IN_BUF_SIZE = 256; // Ring buffer for bytes from C64.

DigitalPin<C64_PA2> pa2Pin(INPUT, LOW);
DigitalPin<C64_PC2> pc2Pin(INPUT, LOW);
DigitalPin<C64_FLAG> flagPin(OUTPUT, LOW);
DigitalPin<DATA_DIR> dataDirPin(OUTPUT, LOW);

#define MAX_MIDI_CHANNEL  16

void readByte();

volatile byte inCmdBuf[IN_BUF_SIZE] = {};
volatile byte inBuf[IN_BUF_SIZE] = {};
volatile byte inCmdBufReadPtr = 0;
volatile byte inBufReadPtr = 0;
volatile byte inBufWritePtr = 0;
volatile byte outBuf[OUT_BUF_SIZE] = {};
void (*volatile isrMode)() = readByte;
struct vesselConfigStruct {
  uint16_t receiveChannelMask;
  uint16_t receiveStatusMask;
  bool nmiEnabled;
  bool nmiStatusOnlyEnabled;
  bool transparent;
  volatile byte outBufWritePtr;
  volatile byte outBufReadPtr;
};
struct vesselConfigStruct vesselConfig;
byte receiveCommandMask[MAX_MIDI_CHANNEL] = {};

void noopCmd() {}
void resetCmd();
void purgeCmd();
void panicCmd();
void versionCmd();
void configFlagsCmd();
void configChannelCmd();
void configStatusCmd();
void configCommandCmd();

void (*cmds[])(void) = {
  resetCmd,
  purgeCmd,
  panicCmd,
  versionCmd,
  configFlagsCmd,
  configChannelCmd,
  configStatusCmd,
  configCommandCmd,
};
void (*cmd)(void) = NULL;
const byte cmdLens[] = {
  0,
  0,
  0,
  0,
  1,
  2,
  2,
  1,
};
volatile byte cmdLen = 0;
const byte maxCmd = sizeof(cmdLens) - 1;

#ifdef STATUS
DigitalPin<STATUS> statusPin(OUTPUT, LOW);
volatile bool status = false;

void inline blink() {
  status = !status;
  statusPin.write(status);
}
#else
#define blink()
#endif

class FakeSerial {
  public:
    void begin(int BaudRate __attribute__((unused))) {
    }
    inline __attribute__((always_inline))
    void qread(byte i) {
      c[n++] = i;
    }
    inline __attribute__((always_inline))
    byte read() {
      return c[--n];
    }
    inline __attribute__((always_inline))
    void write(byte i) {
      outBuf[vesselConfig.outBufReadPtr++] = i;
    }
    inline __attribute__((always_inline))
    unsigned available() {
      return n;
    }
  private:
    volatile byte c[128] = {};
    volatile byte n = 0;
};

FakeSerial fs;

#define URXBUFFER_PUSHBACK1 fs.write(mPacket.byte1);
#define URXBUFFER_PUSHBACK2 {fs.write(mPacket.byte1); fs.write(mPacket.byte2);}
#define URXBUFFER_PUSHBACK3 {fs.write(mPacket.byte1); fs.write(mPacket.byte2); fs.write(mPacket.byte3);}

midiEventPacket_t mPacket;

inline bool usbMidiRx() {
  noInterrupts();
  bool packets = false;
  for (;;) {
  mPacket = MidiUSB.read();
  if (mPacket.header == 0) {
    break;
  }
  packets = true;
  byte cin = GETCIN(mPacket);
  byte len = cin2Len[cin][1];
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
  }}
  interrupts();
  return packets;
}

#define  MIDI_CHANNEL  MIDI_CHANNEL_OMNI
struct VesselSettings : public midi::DefaultSettings {
  // cppcheck-suppress unusedStructMember
  static const bool Use1ByteParsing = true;
  static const int BaudRate = 0;
};

MIDI_CREATE_CUSTOM_INSTANCE(FakeSerial, fs, MIDI, VesselSettings);

#define NMI_WRAP(x) { if (vesselConfig.nmiEnabled) { flagPin.write(HIGH); x; flagPin.write(LOW); } else { x; } }
#define NMI_CMD_WRAP(x) { if (!vesselConfig.nmiStatusOnlyEnabled) { NMI_WRAP(x); } }
#define NMI_MIDI_STATUS_SEND(x) NMI_WRAP(MIDI.x)
#define NMI_MIDI_CMD_SEND(x) NMI_CMD_WRAP(MIDI.x)

#define NMI_STATUS_SEND(statusMsg, handler) if (statusMasked(statusMsg)) { NMI_MIDI_STATUS_SEND(handler) }
#define NMI_CHANNEL_SEND(channel, handler) if (channelMasked(channel)) { NMI_MIDI_CMD_SEND(handler) }
#define NMI_COMMAND_SEND(command, channel, handler) if (commandMasked(command, channel)) { NMI_CHANNEL_SEND(channel, handler) }

inline void handleNoteOn(byte channel, byte note, byte velocity) {
  NMI_COMMAND_SEND(1, channel, sendNoteOn(note, velocity, channel))
}

inline void handleNoteOff(byte channel, byte note, byte velocity) {
  NMI_COMMAND_SEND(2, channel, sendNoteOff(note, velocity, channel))
}

inline void handleAfterTouchPoly(byte channel, byte note, byte pressure) {
  NMI_COMMAND_SEND(3, channel, sendAfterTouch(note, pressure, channel))
}

inline void handleControlChange(byte channel, byte number, byte value) {
  NMI_COMMAND_SEND(4, channel, sendControlChange(number, value, channel))
}

inline void handleProgramChange(byte channel, byte number) {
  NMI_COMMAND_SEND(5, channel, sendProgramChange(number, channel))
}

inline void handleAfterTouchChannel(byte channel, byte pressure) {
  NMI_COMMAND_SEND(6, channel, sendAfterTouch(pressure, channel))
}

inline void handlePitchBend(byte channel, int bend) {
  NMI_COMMAND_SEND(7, channel, sendPitchBend(bend, channel))
}

inline void handleTimeCodeQuarterFrame(byte data) {
  NMI_STATUS_SEND(midi::TimeCodeQuarterFrame, sendTimeCodeQuarterFrame(data))
}

inline void handleSongPosition(unsigned int beats) {
  NMI_STATUS_SEND(midi::SongPosition, sendSongPosition(beats))
}

inline void handleSongSelect(byte songnumber) {
  NMI_STATUS_SEND(midi::SongSelect, sendSongSelect(songnumber))
}

inline void handleTuneRequest() {
  NMI_STATUS_SEND(midi::TuneRequest, sendTuneRequest())
}

inline void handleClock(void) {
  NMI_STATUS_SEND(midi::Clock, sendClock())
}

inline void handleStart(void) {
  NMI_STATUS_SEND(midi::Start, sendStart())
}

inline void handleContinue(void) {
  NMI_STATUS_SEND(midi::Continue, sendContinue())
}

inline void handleStop(void) {
  NMI_STATUS_SEND(midi::Stop, sendStop())
}

inline void handleSystemReset(void) {
  NMI_STATUS_SEND(midi::SystemReset, sendSystemReset())
}

inline bool statusMasked(byte status) {
  uint16_t mask = 1 << (status & 0xf);
  return mask & vesselConfig.receiveStatusMask;
}

inline bool channelMasked(byte channel) {
  uint16_t mask = 1 << (channel - 1);
  return mask & vesselConfig.receiveChannelMask;
}

inline bool commandMasked(byte command, byte channel) {
  return command & (receiveCommandMask[channel - 1]);
}

inline void initPins() {
  for (byte p = 0; p < sizeof(c64Pins); ++p) {
    pinMode(c64Pins[p], INPUT);
  }
  #ifdef STATUS
  statusPin.write(LOW);
  #endif
  flagPin.write(LOW);
}

inline void setInputMode() {
  dataDirPin.write(LOW);
  setDataDirInput();
}

inline void setOutputMode() {
  dataDirPin.write(HIGH);
  setDataDirOutput();
}

inline bool inInputMode() {
  return pa2Pin.read();
}

inline bool inBufWaiting() {
  return inBufReadPtr != inBufWritePtr;
}

inline void drainInBuf() {
  // Avoid buffering, ensure we write direct to UART without waiting.
  //if (inBufWaiting() && uartTxready()) {
  //  uartWrite(inBuf[++inBufWritePtr]);
  //  blink();
  //}
}

inline bool transparentDrainOutBuf() {
  flagPin.write(HIGH);
  if (usbMidiRx()) {
    blink();
    flagPin.write(LOW);
    return true;
  }
  return false;
}

inline bool drainOutBuf() {
  if (fs.available()) {
    MIDI.read();
    return true;
  }
  if (usbMidiRx()) {
    blink();
    return true;
  }
  return false;
}

inline void resetWritePtrs() {
  vesselConfig.outBufWritePtr = 0;
  vesselConfig.outBufReadPtr = 0;
}

inline void inputMode() {
  noInterrupts();
  setInputMode();
  isrMode = readByte;
  interrupts();
  if (vesselConfig.transparent) {
    while (inInputMode()) {
      if (!transparentDrainOutBuf()) {
        drainInBuf();
      }
    }
  } else {
    while (inInputMode()) {
      if (!drainOutBuf()) {
        drainInBuf();
      }
    }
  }
}
  
inline void readByte() {
  volatile byte b = getByte();
  if (b == vesselCmd) {
    inCmdBufReadPtr = 0;
    isrMode = readCmdByte;
  } else {
    inBuf[++inBufReadPtr] = b;
    blink();
  }
}

inline void readCmdByte() {
  byte cmdNo = getByte();
  if (cmdNo > maxCmd) {
    isrMode = readByte;
  } else {
    cmdLen = cmdLens[cmdNo];
    cmd = cmds[cmdNo];
    if (cmdLen) {
      isrMode = readCmdData;
    } else {
      runCmd();
    }
  }
}

inline void versionCmd() {
  NMI_WRAP({
    for (byte i = 0; i < sizeof(versionStr); ++i) {
      fs.write(versionStr[i]);
    }})
}

inline void resetCmd() {
  memset(&vesselConfig, 0, sizeof(vesselConfig));
  MIDI.turnThruOff();
}

inline void purgeCmd() {
  resetWritePtrs();
}

inline void panicCmd() {
  // TODO: not yet implemented.
}

inline void configFlagsCmd() {
  byte configFlags = inCmdBuf[0];
  vesselConfig.nmiEnabled = configFlags & 1;
  if (configFlags & 2) {
    MIDI.turnThruOn();
  } else {
    MIDI.turnThruOff();
  }
  vesselConfig.transparent = configFlags & 4;
  vesselConfig.nmiStatusOnlyEnabled = configFlags & 8;
}

inline uint16_t get2bMask() {
  return (inCmdBuf[0] << 8) + inCmdBuf[1];
}

inline byte getLoNib() {
  return inCmdBuf[0] & 0x0f;
}

inline byte getHighNib() {
  return (inCmdBuf[0] & 0xf0) >> 4;
}

inline void configChannelCmd() {
  vesselConfig.receiveChannelMask = get2bMask();
}

inline void configStatusCmd() {
  vesselConfig.receiveStatusMask = get2bMask();
}

inline void configCommandCmd() {
  byte channel = getLoNib();
  byte mask = getHighNib() & 0x07;
  receiveCommandMask[channel] = mask;
}

inline void runCmd() {
  isrMode = readByte;
  (*cmd)();
}

inline void readCmdData() {
  inCmdBuf[inCmdBufReadPtr++] = getByte();
  if (--cmdLen == 0) {
    runCmd();
  }
}

inline void resetWrite() {
  isrMode = noopCmd;
  resetWritePtrs();
  drainInBuf();
}

inline void outputBytes() {
  writeByte();
  // Last byte, and at least one byte written, reset for next cycle.
  if (--vesselConfig.outBufReadPtr == 0) {
    isrMode = resetWrite;
  }
}

inline void IoIsr() {
  (*isrMode)();
}

inline void writeByte() {
  setByte(outBuf[vesselConfig.outBufWritePtr++]);
}

inline void outputMode() {
  noInterrupts();
  setOutputMode();
  setByte(vesselConfig.outBufReadPtr);
  if (vesselConfig.outBufReadPtr) {
    isrMode = outputBytes;
    blink();
  } else {
    isrMode = resetWrite;
  }
  interrupts();
  while (!inInputMode()) {  };
}

void setup() {
  memset(&vesselConfig, 0, sizeof(vesselConfig));
  initPins();
  initPlatform();
  resetWritePtrs();
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleAfterTouchPoly(handleAfterTouchPoly);
  MIDI.setHandleControlChange(handleControlChange);
  MIDI.setHandleProgramChange(handleProgramChange);
  MIDI.setHandleAfterTouchChannel(handleAfterTouchChannel);
  MIDI.setHandlePitchBend(handlePitchBend);
  MIDI.setHandleStart(handleStart);
  MIDI.setHandleContinue(handleContinue);
  MIDI.setHandleStop(handleStop);
  MIDI.setHandleClock(handleClock);
  MIDI.setHandleSystemReset(handleSystemReset);
  MIDI.setHandleTuneRequest(handleTuneRequest);
  MIDI.setHandleTimeCodeQuarterFrame(handleTimeCodeQuarterFrame);
  MIDI.setHandleSongPosition(handleSongPosition);
  MIDI.setHandleSongSelect(handleSongSelect);
  resetCmd();
  attachInterrupt(digitalPinToInterrupt(C64_PC2), IoIsr, RISING);
  detachInterrupt(digitalPinToInterrupt(C64_PA2));
  while (!inInputMode()) { };
}

void loop() {
  // turnaround time between modes is critical
  for (;;) {
    inputMode();
    outputMode();
  }
}
