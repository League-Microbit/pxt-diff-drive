// diffDrive::WheelCommandTap -- an optional per-tick observer of the
// shaped wheel command MotionEngine is about to hand the kernel.
// Host-portable: no pxt.h, no I2C, no board knowledge, no dependency
// beyond libc and velocity_shaper.h's own Phase enum -- the same
// layering heading_wrap.h/encoder_glitch_armor.h/bus_guard.h already
// establish for small host-portable helpers (see the top-level design
// doc's layer table).
//
// Why this exists (the Cutebot Pro support design doc, "where the
// velocity setpoint comes from"): the vendored kernel does not publish
// what it was last COMMANDED (`Output` carries only MEASURED
// velocity/twist; `Command` is private), so a board sitting below the
// kernel -- the Cutebot Pro's hybrid actuation policy, a later piece of
// work -- has no way to learn the shaped setpoint MotionEngine is
// asking for except by MotionEngine telling it directly. This is that
// seam, and nothing more: it does not know a Cutebot exists, does not
// touch I2C, and does not decide anything.
//
// Contract `motion_engine.cpp` upholds at every one of its own
// `kernel_.drive()`/`kernel_.neutral()` call sites (see that file's own
// call sites -- this header does not enumerate them, so the two lists
// cannot drift against each other):
//   - `onDrive(left, right, phase)` fires ALONGSIDE every
//     `kernel_.drive()` call, never instead of it, with the IDENTICAL
//     (left, right) mm/s MotionEngine derives for that call (the same
//     `velocity - twist` / `velocity + twist` split the kernel itself
//     applies internally -- diffdrive.cpp's own `rawLeft`/`rawRight`)
//     and the SAME tick's `VelocityShaper::Step::phase`.
//   - `onNeutral()` fires alongside every `kernel_.neutral()` call, on
//     every path that makes one -- arrival, deadline, stall, estop, a
//     refused drive, wrong-way, hold expiry, endMove()/cancel, and the
//     nudge/pulse raw-duty paths. A refusal tick can fire BOTH
//     `onDrive()` (for the attempted drive) and `onNeutral()` (for the
//     neutral that follows it in the same tick) -- this mirrors the
//     source exactly rather than trying to collapse it to one call.
//
// No tap installed (the Nezha board's own default, forever) costs one
// null check per call site and changes nothing else -- MotionEngine's
// behaviour with `wheelCommandTap() == nullptr` is byte-identical to
// before this class existed.
#pragma once

#include "velocity_shaper.h"

namespace diffDrive {

class WheelCommandTap {
 public:
  virtual ~WheelCommandTap() = default;

  // The shaped per-wheel command MotionEngine is about to hand
  // kernel_.drive(), plus that same tick's shaper phase.
  virtual void onDrive(float left,   // [mm/s]
                       float right,  // [mm/s]
                       VelocityShaper::Phase phase) = 0;

  // Alongside every kernel_.neutral() call MotionEngine makes.
  virtual void onNeutral() = 0;
};

}  // namespace diffDrive
