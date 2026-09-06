// microbit_i2c_bus.cpp -- the ARM implementation of diffDrive::I2CBus.
//
// The whole of this codebase's dependency on CODAL's I2C peripheral,
// in one place: three call shapes, each with the V1/V2 signature split
// that used to be repeated at every call site in
// `platform/nezha_port.cpp`. Extracting it is what makes that file
// host-compilable (see `platform/i2c_bus.h` for why that matters).
//
// This file is PXT-BOUND by construction and is listed as such in
// tests/DESIGN.md -- it exists precisely so that nothing else has to
// be. It has no logic to test: every line is a forward to `uBit.i2c`.
//
// The V1/V2 split is CODAL's, not ours: codal-microbit-v2 takes
// `uint8_t*`, classic DAL (V1) takes `char*`. `MICROBIT_CODAL` is the
// target's own macro, the same one `nezha_port.cpp` and
// `otos_port.cpp` already switch on.
#include "pxt.h"

#include "i2c_bus.h"

namespace diffDrive {

namespace {

class MicroBitI2CBus final : public I2CBus {
 public:
  int write(uint16_t address, const uint8_t* data, int len) override {
#if MICROBIT_CODAL
    // CODAL's write() does not modify the buffer, but takes a non-const
    // pointer; the const_cast is confined to this one line.
    return uBit.i2c.write(address, const_cast<uint8_t*>(data), len);
#else
    return uBit.i2c.write(
        address, reinterpret_cast<char*>(const_cast<uint8_t*>(data)), len);
#endif
  }

  int read(uint16_t address, uint8_t* data, int len) override {
#if MICROBIT_CODAL
    return uBit.i2c.read(address, data, len);
#else
    return uBit.i2c.read(address, reinterpret_cast<char*>(data), len);
#endif
  }
};

}  // namespace

I2CBus& defaultI2CBus() {
  // Function-local static: constructed on first use, which is after
  // static init in every path (the first `NezhaMotorPort` is built
  // inside `ensure()`'s lazy `new Rig()`), and never destroyed. See
  // `i2c_bus.h`'s own comment on why this is a function.
  static MicroBitI2CBus bus;
  return bus;
}

}  // namespace diffDrive
