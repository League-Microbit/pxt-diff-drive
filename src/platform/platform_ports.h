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
// under CODAL is not free: the context switch saves no FPU registers,
// so a parked fiber's spilled locals -- POINTERS as well as floats --
// can be destroyed by the next fiber that does arithmetic. vfp_guard.h
// carries the mechanism, the measurement behind it, and why the
// vfpSafe* wrappers below are the fix. Never yield here except through
// them.
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
