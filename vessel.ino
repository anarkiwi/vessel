// Vessel

// Copyright 2019 Josh Bailey (josh@vandervecken.com)

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include <Arduino.h>
#include <MIDI.h>
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
DigitalPin<STATUS> statusPin(OUTPUT, LOW);

#define MAX_MIDI_CHANNEL  16

void readByte();

volatile byte inCmdBuf[IN_BUF_SIZE] = {};
volatile byte inBuf[IN_BUF_SIZE] = {};
volatile byte inCmdBufReadPtr = 0;
volatile byte inBufReadPtr = 0;
volatile byte inBufWritePtr = 0;
volatile byte outBuf[OUT_BUF_SIZE] = {};
volatile byte outBufWritePtr = 0;
volatile byte *outBufReadPtr = outBuf;
void (*volatile isrMode)() = readByte;
volatile bool nmiEnabled = false;
volatile bool nmiStatusOnlyEnabled = false;
volatile bool transparent = false;
volatile bool status = false;
volatile uint16_t receiveChannelMask = 0;
volatile uint16_t receiveStatusMask = 0;
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

void inline blink() {
  status = !status;
  statusPin.write(status);
}

class FakeSerial {
  public:
    void begin(int BaudRate __attribute__((unused))) {
      _pending = false;
    }
    inline __attribute__((always_inline))
    void set(byte i) {
      c = i;
      _pending = true;
    }
    inline __attribute__((always_inline))
    byte read() {
      _pending = false;
      return c;
    }
    inline __attribute__((always_inline))
    void write(byte i) {
      outBuf[++(*outBufReadPtr)] = i;
    }
    inline __attribute__((always_inline))
    unsigned available() {
      if (_pending) {
        return 1;
      }
      return 0;
    }
    byte c;
  private:
    bool _pending;
};

FakeSerial fs;

#define  MIDI_CHANNEL  MIDI_CHANNEL_OMNI
struct VesselSettings : public midi::DefaultSettings {
  // cppcheck-suppress unusedStructMember
  static const bool Use1ByteParsing = true;
  static const int BaudRate = 0;
};

MIDI_CREATE_CUSTOM_INSTANCE(FakeSerial, fs, MIDI, VesselSettings);

#define NMI_WRAP(x) { if (nmiEnabled) { flagPin.write(HIGH); } x; }
#define NMI_CMD_WRAP(x) { if (nmiEnabled && !nmiStatusOnlyEnabled) { flagPin.write(HIGH); } x; }
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
  return mask & receiveStatusMask;
}

inline bool channelMasked(byte channel) {
  uint16_t mask = 1 << (channel - 1);
  return mask & receiveChannelMask;
}

inline bool commandMasked(byte command, byte channel) {
  return command & (receiveCommandMask[channel - 1]);
}

inline void initPins() {
  for (byte p = 0; p < sizeof(c64Pins); ++p) {
    pinMode(c64Pins[p], INPUT);
  }
  statusPin.write(LOW);
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
  if (inBufWaiting() && uartTxready()) {
    uartWrite(inBuf[++inBufWritePtr]);
    blink();
  }
}

inline bool drainOutBuf() {
  if (fs.available()) {
    MIDI.read();
    flagPin.write(LOW);
    return true;
  } else if (uartRxready()) {
    if (transparent) {
      NMI_WRAP(fs.write(uartRead()));
      flagPin.write(LOW);
    } else {
      fs.set(uartRead());
    }
    blink();
    return true;
  }
  return false;
}

inline void resetWritePtrs() {
  outBufWritePtr = 0;
  *outBufReadPtr = 0;
}

inline void inputMode() {
  noInterrupts();
  setInputMode();
  isrMode = readByte;
  interrupts();
  while (inInputMode()) {
    if (!drainOutBuf()) {
      drainInBuf();
    }
  }
}
  
inline void readByte() {
  volatile byte b = getByte();
  if (b == vesselCmd) {
    inCmdBufReadPtr = 0;
    inCmdBuf[0] = b;
    isrMode = readCmdByte;
  } else {
    inBuf[++inBufReadPtr] = b;
    blink();
  }
}

inline void readCmdByte() {
  inCmdBuf[++inCmdBufReadPtr] = getByte();
  byte cmdNo = inCmdBuf[1];
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
  flagPin.write(LOW);
}

inline void resetCmd() {
  receiveChannelMask = 0;
  receiveStatusMask = 0;
  memset(receiveCommandMask, 0, sizeof(receiveCommandMask));
  nmiEnabled = false;
  transparent = false;
  nmiStatusOnlyEnabled = false;
  MIDI.turnThruOff();
  purgeCmd();
}

inline void purgeCmd() {
  resetWritePtrs();
}

inline void panicCmd() {
  // TODO: not yet implemented.
}

inline void configFlagsCmd() {
  byte configFlags = inCmdBuf[2];
  nmiEnabled = configFlags & 1;
  if (configFlags & 2) {
    MIDI.turnThruOn();
  } else {
    MIDI.turnThruOff();
  }
  transparent = configFlags & 4;
  nmiStatusOnlyEnabled = configFlags & 8;
}

inline uint16_t get2bMask() {
  return (inCmdBuf[2] << 8) + inCmdBuf[3];
}

inline byte getLoNib() {
  return inCmdBuf[2] & 0x0f;
}

inline byte getHighNib() {
  return (inCmdBuf[2] & 0xf0) >> 4;
}

inline void configChannelCmd() {
  receiveChannelMask = get2bMask();
}

inline void configStatusCmd() {
  receiveStatusMask = get2bMask();
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
  inCmdBuf[++inCmdBufReadPtr] = getByte();
  if (inCmdBufReadPtr > cmdLen) {
    runCmd();
  }
}

inline void outputBytes() {
  writeByte();
  // Last byte, and at least one byte written, reset for next cycle.
  if (outBufWritePtr > *outBufReadPtr) {
    isrMode = noopCmd;
    resetWritePtrs();
  }
}

inline void IoIsr() {
  (*isrMode)();
}

inline void writeByte() {
  setByte(outBuf[outBufWritePtr++]);
}

inline void outputMode() {
  noInterrupts();
  setOutputMode();
  writeByte();
  if (*outBufReadPtr) {
    isrMode = outputBytes;
    blink();
  } else {
    isrMode = noopCmd;
    resetWritePtrs();
  }
  interrupts();
  while (!inInputMode()) {
    drainInBuf();
  }
}

void setup() {
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
