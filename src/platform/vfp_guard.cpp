// vfp_guard.cpp -- see vfp_guard.h for why this exists and why the
// bodies must stay out of line.
#include "vfp_guard.h"

#include "pxt.h"

namespace diffDrive {

// The clobber is what creates the frame: it marks d8-d15 used, so the
// prologue emits `vpush.64 {d8-d15}` before the call below and the
// epilogue emits `vldm sp!, {d8-d15}` after it. Verify with a
// disassembler that both are present and that the call is a `bl` -- a
// `b.w` means the tail call survived and the guard is inert.
__attribute__((noinline)) void vfpSafeSleep(uint32_t ms) {
  fiber_sleep(ms);
  DIFFDRIVE_VFP_BANK_CLOBBER();
}

__attribute__((noinline)) void vfpSafeYield() {
  schedule();
  DIFFDRIVE_VFP_BANK_CLOBBER();
}

}  // namespace diffDrive
