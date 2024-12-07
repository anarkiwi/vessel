#ifdef ARDUINO_ARCH_SAM
#define DISABLE_UART_INT(x) { x->US_IDR = 0xFFFFFFFF; NVIC_ClearPendingIRQ( x ## _IRQn ); NVIC_DisableIRQ( x ## _IRQn ); }

DigitalPin<CONT_DIR> controlDirPin(OUTPUT, LOW);

// TODO: would be cleaner to subclass UARTClass without interrupts or a ringbuffer.
// https://github.com/arduino/ArduinoCore-sam/blob/master/cores/arduino/UARTClass.cpp
// bypass all interrupt usage and do polling with our own ringbuffer.
// inline bool uartTxready() {
//   return USART1->US_CSR & US_CSR_TXRDY;
// }

// inline void uartWrite(byte b) {
//   USART1->US_THR = b;
// }

// inline bool uartRxready() {
//   return USART1->US_CSR & US_CSR_RXRDY;
// }

// inline byte uartRead() {
//   return USART1->US_RHR;
// }

inline bool uartTxready() {
  return true;
}

inline void uartWrite(byte b) {
  UMIDI.write(b);
}

inline bool uartRxready() {
  return UMIDI.available();
}

inline byte uartRead() {
  return UMIDI.read();
}

inline void initPlatform() {
  Serial2.begin(31250);
  while (Serial2.available()) {
    Serial2.read();
  }
  DISABLE_UART_INT(USART2);
  DISABLE_UART_INT(USART1);
  DISABLE_UART_INT(USART0);
  REG_PMC_PCER0 |= (1UL << ID_PIOD); // enable PIO controller.
  controlDirPin.write(LOW);
}

inline void setDataDirInput() {
  REG_PIOD_ODR |= PB_PINS;
  REG_PIOD_OWDR |= PB_PINS;
}

inline void setDataDirOutput() {
  REG_PIOD_OER |= PB_PINS;
  REG_PIOD_OWER |= PB_PINS;
}

inline byte getByte() {
  return REG_PIOD_PDSR; // only need to return LSB
}

inline void setByte(byte b) {
  REG_PIOD_ODSR = b;
}

void IoIsr();

// TODO: custom optimized PIOC handler for PC2 int only.
// add void PIOC_Handler (void) __attribute__ ((weak)); to WInterrupts.c
void PIOC_Handler(void) {
  // TODO: calculate PC2 mask.
  if (PIOC->PIO_ISR & PIO_PC9) {
    IoIsr();
  }
}
#endif

#ifdef ARDUINO_ARCH_SAMD
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

// https://learn.sparkfun.com/tutorials/samd21-minidev-breakout-hookup-guide/setting-up-arduino
// https://learn.sparkfun.com/tutorials/adding-more-sercom-ports-for-samd-boards/all

#define MIDI_RX SCK
#define MIDI_TX MOSI
#define SERCOM SERCOM4
#define VSERCOM sercom4

Uart MidiSerial(&VSERCOM, MIDI_RX, MIDI_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2);

inline bool uartTxready() {
  return VSERCOM.isDataRegisterEmptyUART();
}

inline void uartWrite(byte b) {
  VSERCOM.writeDataUART(b);
}

inline bool uartRxready() {
  return VSERCOM.availableDataUART();
}

inline byte uartRead() {
  return VSERCOM.readDataUART();
}

inline void initPlatform() {
  // see SERCOM::initUART()
  pinPeripheral(MIDI_TX, PIO_SERCOM);
  pinPeripheral(MIDI_RX, PIO_SERCOM);
  pinPeripheral(DATA_DIR, PIO_OUTPUT);
  pinPeripheral(C64_FLAG, PIO_OUTPUT);
  pinPeripheral(C64_PA2, PIO_INPUT);
  MidiSerial.begin(31250);
  SERCOM->USART.INTENCLR.reg = SERCOM_USART_INTENSET_RXC | SERCOM_USART_INTENSET_ERROR;
}

inline void setDataDirInput() {
  PORT->Group[PORTA].DIRCLR.reg = PB_PINS;
}

inline void setDataDirOutput() {
  PORT->Group[PORTA].DIRCLR.reg = PB_PINS;
  PORT->Group[PORTA].DIRSET.reg = PB_PINS;
}

inline byte getByte() {
  return PORT->Group[PORTA].IN.reg >> 16;
}

inline void setByte(byte b) {
  PORT->Group[PORTA].OUT.reg = b << 16;
}
#endif
