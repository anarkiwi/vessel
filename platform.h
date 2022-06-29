DigitalPin<CONT_DIR> controlDirPin(OUTPUT, LOW);

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

inline void initPlatform() {
  Serial2.begin(31250);
  while (Serial2.available()) {
    Serial2.read();
  }
  USART2->US_IDR = 0xFFFFFFFF;
  NVIC_ClearPendingIRQ(USART2_IRQn);
  NVIC_DisableIRQ(USART2_IRQn);
  USART1->US_IDR = 0xFFFFFFFF;
  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_DisableIRQ(USART1_IRQn);
  USART0->US_IDR = 0xFFFFFFFF;
  NVIC_ClearPendingIRQ(USART0_IRQn);
  NVIC_DisableIRQ(USART0_IRQn);
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
