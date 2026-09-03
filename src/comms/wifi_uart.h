// wifi_uart.h -- WifiUartCodal: WifiLink's byte pipe on the nRF52833's
// SECOND UARTE peripheral (NRF_UARTE1), routed to Nezha RJ11 jack J1
// (micro:bit TX=P8, RX=P1 -- the pin map from ELECFREAKS' own
// pxt-PlanetX wifi.ts initWIFI(), confirmed on hardware by the
// radio-robot-elite bench bridge and by nezha-upy's native shim on
// tovez).
//
// UARTE0 is spoken for: it is the USB CDC link CODAL hands out as
// uBit.serial, which SerialTransport owns. UARTE1 is free.
//
// This header is deliberately CODAL-FREE (the NRF52Serial is owned
// through a forward-declared pointer and constructed in begin(), in the
// .cpp) so protocol.h can include it without pulling pxt.h into every
// includer -- the same constraint radio_transport.h honours.
//
// Every method is non-blocking: ASYNC reads and sends only, so nothing
// here can yield and nothing needs the VFP guard
// (the fiber-yield-safety rule).
//
// Two hardware facts a caller must live with (measured during the
// original bring-up, radio-robot-elite docs/knowledge, the 2026-08-08 wifi-
// module-sockets note): the LED matrix refresh and the I2C IRQ guard both
// contend for the IRQs UARTE RX DMA needs, and a received byte can drop
// under that contention. WifiLink's matchers are substring-based and a
// dropped byte in a v6 line costs one nack + host retransmit -- both
// tolerate it; neither should be "fixed" by disabling the display or
// the guard.
#pragma once

#include <cstdint>

#include "wifi_link.h"

namespace codal {
class NRF52Serial;
}

namespace diffDrive {

class WifiUartCodal : public WifiUart {
 public:
  // CODAL's buffer-size setters take a uint8_t: 255 is the true
  // ceiling and a larger request wraps. 250 keeps the same margin
  // SerialTransport's kRingBytes reasoning keeps.
  static constexpr uint8_t kBufferSize = 250;

  WifiUartCodal() : serial_(nullptr) {}

  void begin(uint32_t baud) override;
  uint16_t read(uint8_t* buf, uint16_t cap) override;
  bool write(const uint8_t* data, uint16_t len) override;
  void clearRx() override;

 private:
  codal::NRF52Serial* serial_;
};

}  // namespace diffDrive
