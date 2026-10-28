// position_motor.h -- PositionMotor: the position-move port seam.
//
// A second interface a hardware port may implement BESIDE
// DiffDrive::Motor. It lets the motion engine hand a wheel a relative
// target and let the motor's own position loop run it, instead of
// streaming velocity commands. Pure virtual, libc/C++11 only: no pxt.h,
// no CODAL, no I2C, no timing. The velocity kernel (diffdrive.h /
// diffdrive.cpp) neither includes nor knows about this header.
//
// Models the Nezha brick's built-in position move: the opcode map and
// frame bytes are in the brick opcode reference under docs/knowledge/
// (hardStop() is the brick's hard-stop opcode). Nothing here claims a
// hardware result.
//
// Rounding contract: a port rounds the deltaCounts passed to beginMove()
// to its positionQuantum(). The caller reads the delta actually
// commanded back through lastMoveCounts() when it needs it (the engine
// splits rounding across wheels).
#pragma once

namespace DiffDrive {

class PositionMotor {
 public:
  virtual ~PositionMotor() {}

  // True if this port has a built-in position move.
  virtual bool supportsPositionMove() const = 0;

  // Smallest step the motor honours [counts]; 10 on the brick.
  virtual float positionQuantum() const = 0;

  // Start a relative move of deltaCounts [counts, signed] at degPerSec
  // [deg/s]. Returns false if the port refuses the move.
  virtual bool beginMove(float deltaCounts, float degPerSec) = 0;

  // Stop now and hold the current count (the brick's hard-stop opcode).
  virtual bool hardStop() = 0;

  // A built-in move is in flight on this wheel.
  virtual bool moveLive() const = 0;

  // The wheel is being driven by either path (velocity or position).
  virtual bool driven() const = 0;

  // The delta [counts] actually commanded by the last beginMove(), after
  // rounding to positionQuantum(). 0 before any move.
  virtual float lastMoveCounts() const { return 0.0f; }
};

}  // namespace DiffDrive
