// diffDrive::PositionMove -- one in-flight move run by the motors' own
// position loops (DiffDrive::PositionMotor), the fourth command beside
// Segment, Hold and Nudge. Plain data; MotionEngine plans it
// (beginPositionMove) and ticks it (servicePositionMove). See the
// motion subsystem design doc and the brick opcode reference under
// docs/knowledge/.
#pragma once

#include <cstdint>

namespace diffDrive {

struct PositionMove {
  bool active = false;

  // Origin captured synchronously at begin, from the kernel's Output.
  float posLeft0 = 0.0f, posRight0 = 0.0f;  // [counts]

  // The delta each port actually commanded (PositionMotor::
  // lastMoveCounts(), after quantum rounding); 0 = that wheel was not
  // moved (its share rounded below one quantum).
  float moveLeft = 0.0f, moveRight = 0.0f;  // [counts]

  uint32_t deadline = 0;  // [ms] the caller's timeout backstop

  // Stall test: the last time either wheel moved, and where they were.
  uint32_t progressAt = 0;                      // [ms]
  float lastLeft = 0.0f, lastRight = 0.0f;      // [counts]

  // The most recent move ended inside the arrival window (valid after
  // it ends; cleared by the next begin).
  bool arrived = false;
};

}  // namespace diffDrive
