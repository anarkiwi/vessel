#include "host_sim.h"

namespace hostsim {

std::deque<uint8_t> portIn;
std::vector<uint8_t> portOut;
std::deque<uint8_t> uartIn;
std::vector<uint8_t> uartTx;
bool pinLevel[256] = {};
unsigned long pinWrites[256] = {};

void reset() {
  portIn.clear();
  portOut.clear();
  uartIn.clear();
  uartTx.clear();
  for (int i = 0; i < 256; ++i) {
    pinLevel[i] = false;
    pinWrites[i] = 0;
  }
}

} // namespace hostsim
