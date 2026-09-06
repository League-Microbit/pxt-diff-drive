// vfp_guard.h -- save the callee-saved FPU registers across a yield.
//
// THE HAZARD. This firmware builds with the hardware FPU enabled
// (-mfpu=fpv4-sp-d16 -mfloat-abi=softfp) and CODAL's fiber context
// switch saves only R0-R12/SP/LR -- swap_context contains no VFP
// instruction at all. GCC allocates the callee-saved bank s16-s31
// (= d8-d15) as ordinary spill space, for POINTERS as well as floats,
// so a fiber parked at a yield can have its locals overwritten by the
// next fiber that does arithmetic. MEASURED gopiv 2026-09-01 (pyOCD,
// fault-spin build; forensics in
// docs/knowledge/2026-09-01-codal-does-not-save-fpu-registers-across-fibers.md):
// the protocol fiber parked an object pointer in s17 across its poll
// sleep, a tour fiber's PID wrote a wheel speed over it, and the
// protocol fiber woke and dereferenced float -25.0f as `this` --
// precise data bus error (CFSR 0x8200), board reset.
//
// THE FIX. Declaring d8-d15 clobbered marks them used by the wrapper,
// which obliges AAPCS to save and restore the bank around the yielding
// call on the CALLING FIBER's own stack -- exactly the work
// swap_context omits. Three properties follow:
//
//   1. Every ancestor frame on that fiber is protected, at any depth,
//      because the whole bank is saved unconditionally -- which is why
//      guarding CodalSleeper covers the vendored kernel's encoder
//      settle sleeps without editing the vendored kernel.
//   2. Coverage is monotonic: the save area is per-frame and therefore
//      per-fiber, so an unguarded fiber can lose its own values but can
//      never corrupt a guarded one.
//   3. It is sufficient, not a mitigation: CODAL is non-preemptive, so
//      switches happen only at explicit yields and the set of dangerous
//      moments is finite and enumerable.
//
// noinline is load-bearing for cost and verifiability, not correctness:
// inlined, GCC hoists the save into the enclosing function's prologue
// and then refuses to allocate d8-d15 across the asm, degrading
// allocation through that whole function and leaving no symbol to
// verify. Do not reach for
// __attribute__((target("general-regs-only"))) -- it parses on this
// toolchain and then ICEs as soon as the function touches a float.
#pragma once

#include <stdint.h>

// Active only where there is a hardware FPU to lose. __ARM_FP alone is
// not sufficient -- it is defined on 64-bit ARM hosts too, where this
// header is compiled inertly by the host test harness.
#if defined(__arm__) && defined(__ARM_FP)
#define DIFFDRIVE_VFP_BANK_CLOBBER()                                     \
  __asm__ volatile("" ::: "d8", "d9", "d10", "d11", "d12", "d13", "d14", \
                          "d15")
#else
#define DIFFDRIVE_VFP_BANK_CLOBBER() ((void)0)
#endif

namespace diffDrive {

// fiber_sleep(ms) with the callee-saved FPU bank preserved. THE ONLY
// sanctioned way for this extension to sleep a fiber.
void vfpSafeSleep(uint32_t ms);

// schedule() with the same protection -- a bare scheduling point, no
// timed wait.
void vfpSafeYield();

}  // namespace diffDrive
