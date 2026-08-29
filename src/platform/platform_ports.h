// platform_ports.h -- CODAL implementations of the DiffDrive kernel's
// Clock / Sleeper / FiberLauncher ports for the MakeCode (pxt-microbit)
// target. Mirrors the firmware's platform/microbit implementations:
// every method is one CODAL call.
#pragma once

#include "pxt.h"
#include "../core/diffdrive.h"

namespace diffDrive {

class CodalClock final : public DiffDrive::Clock {
 public:
  uint64_t nowMicros() const override {
    return system_timer_current_time_us();  // [us]
  }
};

class CodalSleeper final : public DiffDrive::Sleeper {
 public:
  void sleepMillis(uint32_t duration) override {  // [ms]
    fiber_sleep(duration);  // cooperative -- yields to other fibers
  }
  void yield() override {
    schedule();  // bare scheduling point, no timed wait
  }
};

class CodalFiberLauncher final : public DiffDrive::FiberLauncher {
 public:
  void launch(void (*entry)(void*), void* context) override {
    create_fiber(entry, context);  // kernel entry never returns
  }
};

}  // namespace diffDrive
