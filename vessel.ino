// Vessel

// Copyright 2019 Josh Bailey (josh@vandervecken.com)

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include <Arduino.h>
#include <MIDI.h>

// TODO: custom optimized PIOC handler for PC2 int only.
// add void PIOC_Handler (void) __attribute__ ((weak)); to WInterrupts.c
void PIOC_Handler(void) {
  // TODO: calculate PC2 mask.
  if (PIOC->PIO_ISR & PIO_PC9) {
    IoIsr();
  }
}

// The C64 can send us a command by sending 0xf9, then another byte specifying the command number, then any other bytes the command requires.
// Each command has a fixed number of bytes expected after the command byte.
const byte vesselCmd = 0xf9;

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

enum IsrModeEnum { ISR_INPUT, ISR_INPUT_CMD_BYTE, ISR_INPUT_CMD_DATA, ISR_OUTPUT, ISR_OUTPUT_DONE };

volatile byte inCmdBuf[IN_BUF_SIZE] = {};
volatile byte inBuf[IN_BUF_SIZE] = {};
volatile byte inCmdBufReadPtr = 0;
volatile byte inBufReadPtr = 0;
volatile byte inBufWritePtr = 0;
volatile byte outBuf[OUT_BUF_SIZE] = {};
volatile byte outBufWritePtr = 0;
volatile byte *outBufReadPtr = outBuf;
volatile IsrModeEnum isrMode = ISR_INPUT;
volatile uint16_t receiveChannelMask = 0;
volatile bool nmiEnabled = false;

void (*cmds[])(void) = {
  configCmd,
  resetCmd,
};
const byte cmdLens[] = {
  3,
  0,
};
byte const *cmdLen = cmdLens;
volatile byte *cmdByte = inCmdBuf + 1;
const byte maxCmd = sizeof(cmdLens) - 1;

class FakeSerial {
  public:
    void begin(int BaudRate) {
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

#define NMI_SEND(x) { if (nmiEnabled) { flagPin.write(HIGH); } MIDI.x;  }
#define NMI_CHANNEL_SEND(x) if (channelMasked(channel)) { NMI_SEND(x) }

inline void handleNoteOn(byte channel, byte note, byte velocity) {
  NMI_CHANNEL_SEND(sendNoteOn(note, velocity, channel))
}

inline void handleNoteOff(byte channel, byte note, byte velocity) {
  NMI_CHANNEL_SEND(sendNoteOff(note, velocity, channel))
}

inline void handleAfterTouchPoly(byte channel, byte note, byte pressure) {
  NMI_CHANNEL_SEND(sendPolyPressure(note, pressure, channel))
}

inline void handleControlChange(byte channel, byte number, byte value) {
  NMI_CHANNEL_SEND(sendControlChange(number, value, channel))
}

inline void handleProgramChange(byte channel, byte number) {
  NMI_CHANNEL_SEND(sendProgramChange(number, channel))
}

inline void handleAfterTouchChannel(byte channel, byte pressure) {
  NMI_CHANNEL_SEND(sendAfterTouch(pressure, channel))
}

inline void handlePitchBend(byte channel, int bend) {
  NMI_CHANNEL_SEND(sendPitchBend(bend, channel))
}

inline void handleTimeCodeQuarterFrame(byte data) {
  NMI_SEND(sendTimeCodeQuarterFrame(data))
}

inline void handleSongPosition(unsigned int beats) {
  NMI_SEND(sendSongPosition(beats))
}

inline void handleSongSelect(byte songnumber) {
  NMI_SEND(sendSongSelect(songnumber))
}

inline void handleTuneRequest() {
  NMI_SEND(sendTuneRequest())
}

inline void handleClock(void) {
  NMI_SEND(sendClock())
}

inline void handleStart(void) {
  NMI_SEND(sendStart())
}

inline void handleContinue(void) {
  NMI_SEND(sendContinue())
}

inline void handleStop(void) {
  NMI_SEND(sendStop())
}

inline void handleSystemReset(void) {
  NMI_SEND(sendSystemReset())
}

inline bool channelMasked(byte channel) {
  uint16_t mask = 1 << (channel - 1);
  return mask & receiveChannelMask;
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
  if (uartTxready() && inBufWaiting()) {
    uartWrite(inBuf[++inBufWritePtr]);
  }
}

inline void drainOutBuf() {
  if (uartRxready()) {
    fs.set(uartRead());
    MIDI.read();
    flagPin.write(LOW);
  }
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
    drainOutBuf();
    drainInBuf();
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
    inBuf[++inBufReadPtr] = getByte();
  }
}

inline void readCmdByte() {
  inCmdBuf[++inCmdBufReadPtr] = getByte();
  if (*cmdByte > maxCmd) {
    isrMode = ISR_INPUT;
  } else {
    isrMode = ISR_INPUT_CMD_DATA;
    cmdLen = cmdLens + *cmdByte;
  }
}

inline void resetCmd() {
  receiveChannelMask = 0;
  nmiEnabled = false;
  MIDI.turnThruOff();
}

inline void configCmd() {
  receiveChannelMask = (inCmdBuf[2] << 8) + inCmdBuf[3];
  byte configFlags = inCmdBuf[4];
  nmiEnabled = configFlags & 1;
  if (configFlags & 2) {
    MIDI.turnThruOff();
  } else {
    MIDI.turnThruOn();
  }
}

inline void readCmdData() {
  inCmdBuf[++inCmdBufReadPtr] = getByte();
  if (inCmdBufReadPtr > *cmdLen) {
    isrMode = ISR_INPUT;
    cmds[*cmdByte]();
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
