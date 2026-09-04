// bus_guard.h -- BusGuard: the shared-I2C-bus ownership guard promoted
// out of tickDrive()'s own inline `stepBusy` flag (code review
// 2026-09-02 RC-01 / CM-03 / BT-04 / BT-05; see the issue titled
// "enforce the one-fiber I2C invariant" under clasi/ for the full
// write-up).
//
// **What this fixes.** `tickDrive()` (src/shims.cpp) serializes
// `kernel.step()` against a second fiber also calling `tickDrive()` --
// a bare `bool` (`Rig::stepBusy`, pre-ticket) checked and set with no
// intervening yield, which is atomic under CODAL's cooperative fiber
// scheduler (a fiber only yields at an explicit sleep/yield call, never
// preemptively). Nothing else that touches the shared I2C bus took
// this same guard: every OTOS shim entry point issued I2C
// unconditionally, `SET rebase`'s OTOS zero ran synchronously on
// whichever fiber called it, and `test/test.ts` ran a free-running
// background sampler with, per its own comment, "NO mutual exclusion"
// against the bus. Any of those landing inside the Nezha encoder's
// select->read settle window destroys that encoder sample (the
// documented Phase-F signature, `src/platform/nezha_port.cpp:376-380`).
// This class is the ONE guard both `tickDrive()` and every OTOS entry
// point now share, so "the bus has exactly one owner at a time" is an
// invariant of the type instead of a convention each call site had to
// remember on its own.
//
// **Why this needs `DiffDrive::Sleeper`, unlike `encoder_glitch_armor.h`
// / `heading_wrap.h` alongside it in this directory.** Those two are
// pure functions of their own inputs and take no port at all. This
// class's whole job is to spin-wait through the SAME sleeper the
// kernel itself paces on (so a caller blocked here still yields to
// other fibers instead of busy-spinning the CPU) -- that requires the
// `DiffDrive::Sleeper` interface `src/core/diffdrive.h` already
// defines. `diffdrive.h` itself depends on nothing but `<cstdint>`
// (its own header comment), so pulling it in here does not cost this
// header its host portability: no `pxt.h`, no CODAL, compiles at both
// the host suite's `-std=c++20` and the embedded targets' `-std=c++11`
// (`tests/host/test_cxx11_syntax_gate.py`, extended for this file by
// `bus_guard_syntax_check.cpp` -- this header has no natural `.cpp` of
// its own, same reason `heading_wrap_syntax_check.cpp` exists).
//
// **Manual acquire/release, not RAII.** The codebase's own style for
// this exact flag is a manual acquire/release pair (`tickDrive()`'s
// pre-extraction `stepBusy` loop), and every call site this ticket
// wires up is a short, single-exit function body where the acquire and
// release bracket the whole I2C-touching span with no early return in
// between -- an RAII scope guard would buy nothing a plain pair of
// calls doesn't already give here, and it would be one more concept a
// student-facing extension's own contributors need to learn to read
// `shims.cpp`. Documented per the ticket's own explicit implementer's-
// judgment allowance.
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
  // Byte-identical to tickDrive()'s own pre-extraction inline loop
  // (src/shims.cpp, pre-ticket lines 663-666): spins, sleeping 1 ms
  // between checks, until the bus is free, then claims it for the
  // caller. A second caller -- on a different fiber, or (for the OTOS
  // entry points) the SAME fiber reentering while another one of them
  // is mid-transaction -- just waits; it never races the I2C traffic
  // this guards.
  void acquire(DiffDrive::Sleeper& sleeper) {
    while (busy_) {
      sleeper.sleepMillis(1);
    }
    busy_ = true;
  }

  // Releases the bus. The caller must hold it (i.e. have returned from
  // a matching acquire()) -- release() does not itself check this,
  // matching stepBusy's own pre-extraction contract (a bare flag write,
  // no defensive re-check).
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
