#ifdef ARDUINO_ARCH_SAM
// PC9 is 41
#define C64_PC2 41
#define C64_PA2 40
// 74 AKA PA25/MISO
#define C64_FLAG 74
#define DATA_DIR 44
#define CONT_DIR 43
#define STATUS MOSI

#define C64_PB0 25
#define C64_PB1 26
#define C64_PB2 27
#define C64_PB3 28
#define C64_PB4 14
#define C64_PB5 15
#define C64_PB6 29
#define C64_PB7 11

const uint32_t PB_PINS = 0xff;
#endif

#ifdef ARDUINO_ARCH_SAMD
#define C64_PA2 A5 // PB02
#define C64_FLAG PIN_LED_RXL // PB03
#define DATA_DIR A1 // PB08
#define C64_PC2 A2 // PB09

#define C64_PB0 11 // PA16
#define C64_PB1 13 // PA17
#define C64_PB2 10 // PA18
#define C64_PB3 12 // PA19
#define C64_PB4 6 // PA20
#define C64_PB5 7 // PA21
#define C64_PB6 SDA // PA22
#define C64_PB7 SCL // PA23

const uint32_t PB_PINS = 0x00FF0000;
#endif
