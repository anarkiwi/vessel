// Vessel

// Copyright 2019 Josh Bailey (josh@vandervecken.com)

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include <Arduino.h>
#include <MIDI.h>

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

// TODO: custom optimized PIOC handler for PC2 int only.
// add void PIOC_Handler (void) __attribute__ ((weak)); to WInterrupts.c
void PIOC_Handler(void) {
  // TODO: calculate PC2 mask.
  if (PIOC->PIO_ISR & PIO_PC9) {
    IoIsr();
  }
}

// The C64 can send us a command by sending a reserved cmd byte, then another byte specifying the command number, then any other bytes the command requires.
// Each command has a fixed number of bytes expected after the command byte.
const byte vesselCmd = 0xfd;

#define PINDESC(pin)      g_APinDescription[pin].ulPin
#define PINPPORT(pin, PPIO)       g_APinDescription[pin].pPort->PPIO
#define DISABLE_PERIPHERAL_PIN(pin) g_APinDescription[pin].pPort->PIO_PER = g_APinDescription[pin].ulPin;

template<uint8_t pin>
class DigitalPin {
 public:
  DigitalPin(bool mode, bool value) {
    pinMode(pin, mode);
    write(value);
  }
  inline __attribute__((always_inline))
  void high() { write(true); }
  inline __attribute__((always_inline))
  void low() { write(false); }
  inline __attribute__((always_inline))
  bool read() const {
    return PINPPORT(pin, PIO_PDSR) & PINDESC(pin);
  }
  inline __attribute__((always_inline))
  void write(bool value) {
    if (value) {
      PINPPORT(pin, PIO_SODR) = PINDESC(pin);
    } else {
      PINPPORT(pin, PIO_CODR) = PINDESC(pin);
    }
  }
};

// PC9 is 41
#define C64_PC2 41
#define C64_PA2 40
// 74 AKA PA25/MISO
#define C64_FLAG 74
#define DATA_DIR 44
#define CONT_DIR 43

#define C64_PB0 25
#define C64_PB1 26
#define C64_PB2 27
#define C64_PB3 28
#define C64_PB4 14
#define C64_PB5 15
#define C64_PB6 29
#define C64_PB7 11

const byte c64Pins[] = {C64_PB0, C64_PB1, C64_PB2, C64_PB3, C64_PB4, C64_PB5, C64_PB6, C64_PB7};
const uint32_t PB_PINS = 0xff; // PIO mask (PIO takes 32 bits, we want the lowest 8 bits)
const uint16_t OUT_BUF_SIZE = 256; // Ring buffer for bytes to send to C64
const uint16_t IN_BUF_SIZE = 256; // Ring buffer for bytes from C64.

DigitalPin<C64_PA2> pa2Pin(INPUT, LOW);
DigitalPin<C64_PC2> pc2Pin(INPUT, LOW);
DigitalPin<C64_FLAG> flagPin(OUTPUT, LOW);
DigitalPin<DATA_DIR> dataDirPin(OUTPUT, LOW);
DigitalPin<CONT_DIR> controlDirPin(OUTPUT, LOW);
DigitalPin<MOSI> statusPin(OUTPUT, LOW);

enum IsrModeEnum { ISR_INPUT, ISR_INPUT_CMD_BYTE, ISR_INPUT_CMD_DATA, ISR_OUTPUT, ISR_OUTPUT_DONE };

#define MAX_MIDI_CHANNEL  16

volatile byte inCmdBuf[IN_BUF_SIZE] = {};
volatile byte inBuf[IN_BUF_SIZE] = {};
volatile byte inCmdBufReadPtr = 0;
volatile byte inBufReadPtr = 0;
volatile byte inBufWritePtr = 0;
volatile byte outBuf[OUT_BUF_SIZE] = {};
volatile byte outBufWritePtr = 0;
volatile byte *outBufReadPtr = outBuf;
volatile IsrModeEnum isrMode = ISR_INPUT;
volatile bool nmiEnabled = false;
volatile bool transparent = false;
volatile bool status = false;
volatile uint16_t receiveChannelMask = 0;
volatile uint16_t receiveStatusMask = 0;
byte receiveCommandMask[MAX_MIDI_CHANNEL] = {};

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
  if (status) {
    status = LOW;
  } else {
    status = HIGH;
  }
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
#define NMI_MIDI_SEND(x) NMI_WRAP(MIDI.x)

#define NMI_STATUS_SEND(statusMsg, handler) if (statusMasked(statusMsg)) { NMI_MIDI_SEND(handler) }
#define NMI_CHANNEL_SEND(channel, handler) if (channelMasked(channel)) { NMI_MIDI_SEND(handler) }
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

// TODO: would be cleaner to subclass UARTClass without interrupts or a ringbuffer.
// https://github.com/arduino/ArduinoCore-sam/blob/master/cores/arduino/UARTClass.cpp
// bypass all interrupt usage and do polling with our own ringbuffer.
inline bool uartTxready() {
  return USART1->US_CSR & US_CSR_TXRDY;
}

inline void uartWrite(byte b) {
  USART1->US_THR = b;
}

inline bool uartRxready() {
  return USART1->US_CSR & US_CSR_RXRDY;
}

inline byte uartRead() {
  return USART1->US_RHR;
}

inline void disableUartInts() {
  USART2->US_IDR = 0xFFFFFFFF;
  NVIC_ClearPendingIRQ(USART2_IRQn);
  NVIC_DisableIRQ(USART2_IRQn);
  USART1->US_IDR = 0xFFFFFFFF;
  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_DisableIRQ(USART1_IRQn);
  USART0->US_IDR = 0xFFFFFFFF;
  NVIC_ClearPendingIRQ(USART0_IRQn);
  NVIC_DisableIRQ(USART0_IRQn);
}

inline void disablePeripherals() {
  for (byte p = 0; p < sizeof(c64Pins); ++p) {
    DISABLE_PERIPHERAL_PIN(c64Pins[p]);
  }
  DISABLE_PERIPHERAL_PIN(C64_PA2);
  DISABLE_PERIPHERAL_PIN(C64_PC2);
  DISABLE_PERIPHERAL_PIN(C64_FLAG);
}

inline bool inInputMode() {
  return pa2Pin.read();
}

inline void setInputMode() {
  dataDirPin.write(LOW);
  REG_PIOD_ODR |= PB_PINS;
  REG_PIOD_OWDR |= PB_PINS;
}

inline void setOutputMode() {
  dataDirPin.write(HIGH);
  REG_PIOD_OER |= PB_PINS;
  REG_PIOD_OWER |= PB_PINS;
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
  isrMode = ISR_INPUT;
  interrupts();
  while (inInputMode()) {
    if (!drainOutBuf()) {
      drainInBuf();
    }
  }
}
  
byte getByte() {
  return REG_PIOD_PDSR; // only need to return LSB
}

inline void readByte() {
  volatile byte b = getByte();
  if (b == vesselCmd) {
    inCmdBufReadPtr = 0;
    inCmdBuf[0] = b;
    isrMode = ISR_INPUT_CMD_BYTE;
  } else {
    inBuf[++inBufReadPtr] = b;
    blink();
  }
}

inline void readCmdByte() {
  inCmdBuf[++inCmdBufReadPtr] = getByte();
  byte cmdNo = inCmdBuf[1];
  if (cmdNo > maxCmd) {
    isrMode = ISR_INPUT;
  } else {
    cmdLen = cmdLens[cmdNo];
    cmd = cmds[cmdNo];
    if (cmdLen) {
      isrMode = ISR_INPUT_CMD_DATA;
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
  isrMode = ISR_INPUT;
  (*cmd)();
}

inline void readCmdData() {
  inCmdBuf[++inCmdBufReadPtr] = getByte();
  if (inCmdBufReadPtr > cmdLen) {
    runCmd();
  }
}

inline void IoIsr() {
  switch (isrMode) {
    case ISR_INPUT:
      { 
        readByte();
      }
      break;
    case ISR_INPUT_CMD_BYTE:
      {
        readCmdByte();
      }
      break;
    case ISR_INPUT_CMD_DATA:
      {
        readCmdData();
      }
      break;
    case ISR_OUTPUT:
      {
        writeByte();
        // Last byte, and at least one byte written, reset for next cycle.
        if (outBufWritePtr > *outBufReadPtr) {
          isrMode = ISR_OUTPUT_DONE;
          resetWritePtrs();
        }
      }
      break;
    case ISR_OUTPUT_DONE:
      break;
    default:
      break;
  }
}

inline void setByte(byte b) {
  REG_PIOD_ODSR = b;
}

inline void writeByte() {
  setByte(outBuf[outBufWritePtr++]);
}

inline void outputMode() {
  noInterrupts();
  setOutputMode();
  writeByte();
  if (*outBufReadPtr) {
    isrMode = ISR_OUTPUT;
    blink();
  } else {
    isrMode = ISR_OUTPUT_DONE;
    resetWritePtrs();
  }
  interrupts();
  while (!inInputMode()) {
    drainInBuf();
  }
}

void dummyIsr() {
}

void setup() {
  statusPin.write(LOW);
  SerialUSB.begin(115200);
  Serial2.begin(31250);
  while (Serial2.available()) {
    Serial2.read();
  }
  disableUartInts();
  disablePeripherals();
  REG_PMC_PCER0 |= (1UL << ID_PIOD); // enable PIO controller.
  controlDirPin.write(LOW);
  flagPin.write(LOW);
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
  attachInterrupt(digitalPinToInterrupt(C64_PC2), dummyIsr, RISING);
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
