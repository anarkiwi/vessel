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

const uint32_t PB_PINS = 0xff; // PIO mask (PIO takes 32 bits, we want the lowest 8 bits)
