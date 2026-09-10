// wifi_flash_port.h -- WifiFlashPort: the thin platform seam between
// WifiCredentialStore (src/comms/wifi_credential_store.h, host-portable)
// and real flash, mirroring the WifiUart/WifiLink split
// (src/comms/wifi_link.h / wifi_uart.{h,cpp}): the abstract interface
// lives here, CODAL-free, so WifiCredentialStore and its host tests
// never see pxt.h; WifiFlashPortCodal's method BODIES (this header only
// declares them) live in wifi_flash_port.cpp, the one non-host-portable
// translation unit this ticket adds.
//
// == What this seam abstracts ==
//
// Not a generic flash driver -- just enough for ONE dedicated page of
// fixed-size records: read some bytes back (flash is memory-mapped, so
// this is a plain copy, never fails), write some bytes through the
// erase-merge path (so a caller never has to reason about "does this
// region need erasing first" -- see writePage()'s comment), and erase
// the whole page outright. WifiCredentialStore never touches an
// address; it only ever deals in offsets from 0..kPageBytes.
//
// == The page address is a candidate, not a confirmed answer ==
//
// `MICROBIT_STORAGE_PAGE` (0x0007F000) is `uBit.storage`'s
// (KeyValueStorage's) page.
// `MICROBIT_DEFAULT_SCRATCH_PAGE` (0x0007E000) is BOTH the flash
// driver's own erase-merge scratch page (MicroBitFlash::flash_write()'s
// default `scratch_addr`) AND `MICROBIT_APP_REGION_END` -- where the
// compiled program's own region is understood to end. Both addresses
// read from `built/dockercodal/libraries/codal-microbit-v2/inc/
// MicroBitConfig.h`, a source reading, not a hardware measurement.
// `WifiFlashPortCodal::kFlashPageAddress` below picks the next page
// down and is a NAMED CONSTANT precisely so a future firmware region
// shuffle cannot silently collide with it -- but whether that address
// actually survives `mbdeploy deploy` (which erases pages) and a power
// cycle is UNVERIFIED here -- confirming it is real-hardware work, not
// this file's.
#pragma once

#include <cstdint>

namespace diffDrive {

class WifiFlashPort {
 public:
  virtual ~WifiFlashPort() {}

  // How many bytes this seam promises the store, starting at offset 0.
  // 4096 -- one nRF52833 flash page (`NRF_FICR->CODEPAGESIZE`, read at
  // runtime by the vendored `MICROBIT_CODEPAGESIZE` macro; documented
  // as a fixed 4 KiB page on this chip in Nordic's own product
  // specification, not independently re-measured here). Every
  // implementation -- real or fake -- must honor this size; a caller
  // (WifiCredentialStore) static_asserts its own on-flash footprint
  // against it.
  static constexpr uint32_t kPageBytes = 4096;

  // Copies `length` bytes starting at `offset` bytes into the page.
  // Never fails: flash is memory-mapped, so this is a plain read of
  // whatever physical content is there -- all-0xFF for an
  // erased/never-written page, which WifiCredentialStore's own record
  // format treats as "empty," not corrupted (see its header comment).
  // `offset + length` beyond kPageBytes reads nothing and leaves
  // `buffer` untouched -- degrade safely rather than read outside the
  // page.
  virtual void read(uint32_t offset, uint8_t* buffer, uint32_t length) const = 0;

  // Writes `length` bytes starting at `offset` bytes into the page,
  // through the erase-merge write path (MicroBitFlash::flash_write() on
  // the real implementation) -- NOT a raw write that assumes no erase
  // is needed. That means a caller can freely rewrite bytes that need
  // bits set back to 1 (e.g. clearing an occupied slot) without ever
  // managing an erase itself; the port does the erase-preserve-rewrite
  // dance generically. Returns false (and leaves flash unchanged) for
  // an out-of-range `offset`/`length` or on a write failure -- never
  // partial.
  virtual bool write(uint32_t offset, const uint8_t* buffer, uint32_t length) = 0;

  // Erases the whole page: every byte reads back as 0xFF afterward.
  // Returns false on failure -- the real implementation wraps
  // codal::MicroBitFlash::erase_page(), which reports no failure of its
  // own (void return), so WifiFlashPortCodal always returns true; a
  // fake implementation is free to simulate a failure for its own
  // tests.
  virtual bool erasePage() = 0;
};

// The CODAL-coupled implementation. Declared here (host-portable --
// nothing below reaches pxt.h) so WifiCredentialStore's production
// composition root can name the type; every method body lives in
// wifi_flash_port.cpp, which does reach pxt.h, exactly the split
// WifiUartCodal (wifi_uart.h/.cpp) already uses.
class WifiFlashPortCodal final : public WifiFlashPort {
 public:
  // UNVERIFIED on real hardware (see this file's header comment
  // above): one nRF52833 flash page below MICROBIT_DEFAULT_SCRATCH_PAGE
  // (0x0007E000 - 0x1000 = 0x0007D000), chosen to sit outside both
  // MICROBIT_STORAGE_PAGE (0x0007F000, uBit.storage's) and
  // MICROBIT_DEFAULT_SCRATCH_PAGE (0x0007E000, the flash driver's own
  // erase-merge scratch page and MICROBIT_APP_REGION_END). 0x1000 is
  // this chip's documented CODEPAGESIZE (see kPageBytes above), used
  // here as a literal because MICROBIT_CODEPAGESIZE itself is a
  // runtime register read and so cannot appear in a constexpr.
  static constexpr uint32_t kFlashPageAddress = 0x0007D000;

  void read(uint32_t offset, uint8_t* buffer, uint32_t length) const override;
  bool write(uint32_t offset, const uint8_t* buffer, uint32_t length) override;
  bool erasePage() override;
};

}  // namespace diffDrive
