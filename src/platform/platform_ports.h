// platform_ports.h -- CODAL implementations of the DiffDrive kernel's
// Clock / Sleeper / FiberLauncher ports for the MakeCode (pxt-microbit)
// target. Mirrors the firmware's platform/microbit implementations:
// every method is one CODAL call.
#pragma once

#include "pxt.h"
#include "../core/diffdrive.h"
#include "vfp_guard.h"

namespace diffDrive {

class CodalClock final : public DiffDrive::Clock {
 public:
  uint64_t nowMicros() const override {
    return system_timer_current_time_us();  // [us]
  }
};

// HAZARD -- READ BEFORE CHANGING EITHER METHOD BELOW.
//
// These two calls are where this extension yields the CPU, and a yield
// under CODAL is not free. The build enables the hardware FPU
// (-mfpu=fpv4-sp-d16 -mfloat-abi=softfp) but CODAL's swap_context saves
// only R0-R12/SP/LR -- it contains no VFP instructions at all. GCC uses
// the callee-saved bank s16-s31 (= d8-d15) as ordinary spill space, for
// POINTERS as well as floats, so whatever a parked fiber left there is
// destroyed by the next fiber that does arithmetic.
//
// MEASURED gopiv 2026-09-01 (pyOCD on a fault-spin build): the protocol
// fiber parked an object pointer in s17 across its poll sleep, a tour
// fiber's PID wrote a wheel speed over it, and the protocol fiber woke
// and dereferenced float -25.0f as `this`. Precise bus error, board
// reset. See the yield-discipline invariant in this package's design
// notes, and the knowledge article under docs/, for the full forensics.
//
// This class is also the single choke point through which the VENDORED
// kernel yields: DifferentialDrive::step()'s two encoder settle sleeps
// reach it by true indirect virtual call. Guarding here therefore covers
// core/diffdrive.cpp without editing it -- which is the only reason that
// file can stay untouched. Do not bypass it by calling fiber_sleep()
// from elsewhere.
class CodalSleeper final : public DiffDrive::Sleeper {
 public:
  void sleepMillis(uint32_t duration) override {  // [ms]
    vfpSafeSleep(duration);  // cooperative -- yields to other fibers
  }
  void yield() override {
    vfpSafeYield();  // bare scheduling point, no timed wait
  }
};

class CodalFiberLauncher final : public DiffDrive::FiberLauncher {
 public:
  void launch(void (*entry)(void*), void* context) override {
    create_fiber(entry, context);  // kernel entry never returns
  }
};

}  // namespace diffDrive
