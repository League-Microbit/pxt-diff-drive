// bus_guard.h -- BusGuard: the ONE shared-I2C-bus ownership guard.
//
// **The invariant.** The bus has exactly one owner at a time. Any I2C
// transaction landing inside the Nezha encoder's select->read settle
// window destroys that encoder sample (the Phase-F signature,
// `src/platform/nezha_port.cpp`), so `tickDrive()`'s `kernel.step()`
// and every OTOS entry point -- the shims, `SET rebase`'s OTOS zero,
// `test/test.ts`'s background sampler -- must all take THIS guard
// rather than each call site remembering a convention of its own.
//
// A bare `bool` is enough: it is checked and set with no intervening
// yield, and CODAL's fiber scheduler is cooperative (a fiber yields
// only at an explicit sleep/yield call, never preemptively).
//
// **Why this one takes `DiffDrive::Sleeper` when its neighbours take
// no port at all.** `encoder_glitch_armor.h` and `heading_wrap.h` are
// pure functions of their inputs; this class must spin-wait through
// the SAME sleeper the kernel paces on, so a blocked caller yields to
// other fibers instead of busy-spinning the CPU. `diffdrive.h`, which
// defines that interface, depends on nothing but `<cstdint>`, so this
// header stays host-portable: no `pxt.h`, no CODAL, and it compiles at
// both `-std=c++20` and `-std=c++11` (`bus_guard_syntax_check.cpp`
// carries it into the syntax gate -- it has no natural `.cpp`).
//
// **Manual acquire/release, not RAII**: every call site is a short,
// single-exit body whose acquire/release bracket the whole
// I2C-touching span with no early return in between, so a scope guard
// would add a concept without removing a hazard.
//
// Host-tested: tests/host/test_bus_guard.py (scripts
// FakeSleeper::onSleep to release the guard mid-spin and confirms
// acquire() does not return before that).
#pragma once

#include <cstdint>

#include "diffdrive.h"

namespace diffDrive {

class BusGuard {
 public:
  // Spins, sleeping 1 ms between checks, until the bus is free, then
  // claims it for the caller. A second caller -- on a different fiber,
  // or (for the OTOS entry points) the SAME fiber reentering while
  // another one of them is mid-transaction -- just waits; it never
  // races the I2C traffic this guards.
  void acquire(DiffDrive::Sleeper& sleeper) {
    while (busy_) {
      sleeper.sleepMillis(1);
    }
    busy_ = true;
  }

  // Releases the bus. The caller must hold it (i.e. have returned from
  // a matching acquire()) -- release() is a bare flag write and does
  // not itself check that.
  void release() { busy_ = false; }

  // Non-blocking peek at ownership: true iff some caller is currently
  // between an acquire() and its matching release(). Safe to read from
  // any fiber with no locking of its own -- CODAL's cooperative
  // scheduler only switches fibers at an explicit yield/sleep call, so
  // a plain read here can never race a concurrent write to busy_ the
  // way it would under preemption. Lets a caller that must never block
  // (a stop request that has to stay tick-independent) choose to defer
  // its own I2C write instead of spinning through acquire().
  bool held() const { return busy_; }

 private:
  bool busy_ = false;
};

}  // namespace diffDrive
