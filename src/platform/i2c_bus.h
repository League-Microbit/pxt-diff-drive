// i2c_bus.h -- diffDrive::I2CBus: the ONE seam between a port's wire
// protocol and the physical bus.
//
// Why this exists. `platform/nezha_port.cpp` used to call
// `uBit.i2c.write()/read()` directly, which pulled `pxt.h` in through
// `nezha_port.h` and made the whole translation unit un-host-compilable
// (tests/DESIGN.md "Translation units nothing on the host compiles" --
// this file removes `platform/nezha_port.cpp` from that list). The cost
// of that was not abstract: the port's shaping layer -- the sigma-delta
// duty quantizer, the output deadband, the write throttle, and above
// all the 100 ms REVERSAL DWELL -- could not be simulated at all,
// because every host harness substitutes `FakeMotor` at the
// `DiffDrive::Motor` interface, which sits ABOVE all of it.
//
// That mattered. A pivot reverses exactly one wheel, so
// `NezhaMotorPort::writeShapedDuty()`'s reversal dwell holds that wheel
// at commanded zero for ~100 ms while the other starts immediately --
// and no sim in this repo could see it. MEASURED tovez 2026-09-05,
// reports/tovez-square-tour-20260905/square_vel.csv: on every one of
// the square tour's four pivots the reversing wheel trails the forward
// wheel by roughly that interval (frame 0.22 s of pivot 1: -26 mm/s
// against +82 mm/s on a +-100 command).
//
// This interface is the same shape as radio-robot-elite's
// `Hal::I2CBus` (source/hal/i2c_bus.h there), for the same reason and
// with the same two-implementation split: one ARM implementation
// wrapping the real bus (`platform/microbit_i2c_bus.cpp`), one
// simulated implementation that parses the bytes firmware actually
// wrote and answers with encoder bytes (tests/host/). A simulator that
// RESPONDS to the real frame cannot desync from the firmware the way a
// predictor can.
//
// Deliberately NOT modelled here: `repeated`/`preClear`/`postClear`.
// Those exist on CODAL's own bus to schedule real clearance timing,
// which is the ARM implementation's private concern; nothing in this
// codebase's wire protocol varies on them.
#pragma once

#include <cstdint>

namespace diffDrive {

class I2CBus {
 public:
  virtual ~I2CBus() {}

  // Both return 0 on success, non-zero on error -- CODAL's own
  // MICROBIT_OK convention, which every caller in this tree already
  // tests against (`status != 0` means the transaction failed and the
  // sample must be treated as absent, not as zero).
  //
  // `address` is the 8-BIT (already left-shifted) address, matching
  // what `uBit.i2c` takes and what every call site here passes
  // (`kAddress << 1`) -- not the 7-bit device address.
  virtual int write(uint16_t address, const uint8_t* data, int len) = 0;
  virtual int read(uint16_t address, uint8_t* data, int len) = 0;
};

// The process-wide bus a port binds to when its constructor is not
// given one explicitly.
//
// It is a FUNCTION, not a global object, for two reasons. First, the
// static-initialization order problem: `Rig`'s two `NezhaMotorPort`
// members take this as a default argument, and `Rig` is constructed
// lazily inside `ensure()` (shims.cpp) at first motion command, long
// after static init -- but a global bus OBJECT would still have to be
// constructed before them, which nothing guarantees across translation
// units. Second, it keeps the default-argument form working, so
// `NezhaMotorPort left{1, -1}` still parses exactly as before and
// `tools/make_deploy.py`'s per-robot motor bake (`_MOTOR_BAKE_RES`,
// which matches those two literals by regex) is untouched.
//
// Defined once per build flavour: `platform/microbit_i2c_bus.cpp` on
// the target, the host harness's own sim bus on the host.
I2CBus& defaultI2CBus();

}  // namespace diffDrive
