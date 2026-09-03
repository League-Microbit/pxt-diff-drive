// wifi_uart.cpp -- see wifi_uart.h. Talks to a second NRF52Serial on
// NRF_UARTE1 directly, the same way serial_transport.cpp talks to
// uBit.serial: one small CODAL-facing leaf.
#include "wifi_uart.h"

#include "pxt.h"

using namespace pxt;

namespace diffDrive {

void WifiUartCodal::begin(uint32_t baud) {
  if (serial_ == nullptr) {
    // Constructed lazily, never as a global: a global NRF52Serial would
    // be built before uBit.init() brings the pins up (the same reason
    // Protocol is a lazy singleton -- see protocol.h). J1 = TX P8 / RX
    // P1 (see this file's header).
    serial_ = new NRF52Serial(uBit.io.P8, uBit.io.P1, NRF_UARTE1);
    serial_->setRxBufferSize(kBufferSize);
    serial_->setTxBufferSize(kBufferSize);
  }
  serial_->setBaudrate(static_cast<int>(baud));
}

uint16_t WifiUartCodal::read(uint8_t* buf, uint16_t cap) {
  if (serial_ == nullptr || cap == 0) return 0;
  // ASYNC: whatever the DMA ring already holds; never waits for more.
  const int n = serial_->read(buf, static_cast<int>(cap), ASYNC);
  if (n <= 0) return 0;  // negative is CODAL's "no data" code, not an error
  return static_cast<uint16_t>(n);
}

bool WifiUartCodal::write(const uint8_t* data, uint16_t len) {
  if (serial_ == nullptr) return false;
  if (len == 0) return true;
  // WHOLE-BUFFER-OR-NOTHING (WifiUart::write()'s contract): check the
  // free room FIRST, so a full ring costs one honest retry instead of
  // putting a truncated AT command on the wire.
  if (static_cast<int>(kBufferSize) - serial_->txBufferedSize() < static_cast<int>(len)) {
    return false;
  }
  serial_->send(const_cast<uint8_t*>(data), static_cast<int>(len), ASYNC);
  return true;
}

void WifiUartCodal::clearRx() {
  if (serial_ != nullptr) serial_->clearRxBuffer();
}

}  // namespace diffDrive
