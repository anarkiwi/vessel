#define PINDESC(pin)      g_APinDescription[pin].ulPin
#define PINPPORT(pin, PPIO)       g_APinDescription[pin].pPort->PPIO

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
