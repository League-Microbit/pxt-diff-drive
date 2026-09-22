// board_nezha.cpp -- the Nezha half of the board-composition seam
// (board.h), compiled only when DIFFDRIVE_BOARD selects
// DIFFDRIVE_BOARD_NEZHA (the default -- see board.h). Owns exactly the
// things that leak the Nezha board above the DiffDrive::Motor port:
// the concrete two-port construction that used to be inline in
// shims.cpp's Rig (see NezhaBoard below for the exact, unmoved lines),
// the `configure motor` wiring hooks, the diag-ordinal readback, and
// the fault-context emergency-stop frame (moved verbatim from
// nezha_port.cpp -- see that file's own comment for why it must build
// with no object, fiber, scheduler or kernel state in reach).
//
// Host-compilability note, same shape as nezha_port.cpp's own: this
// file reaches `pxt.h` ONLY inside `#ifndef DIFFDRIVE_HOST_BUILD`, for
// the fault-context frame's real `uBit.i2c.write()`. Unlike
// nezha_port.cpp, though, nothing else here is host-LINKABLE as
// written: `NezhaBoard` constructs its two `NezhaMotorPort`s through
// the DEFAULT bus argument (`defaultI2CBus()`), which is defined only
// by the target's `platform/microbit_i2c_bus.cpp` -- there is no host
// definition, by design (i2c_bus.h's own comment). That default-bus
// construction is this file's whole job (it is what "the exact
// construction that used to be inline in shims.cpp's Rig" means), so
// this translation unit stays hex-checkpoint-only in its entirety, the
// same as shims.cpp itself. The Nezha-specific COMPUTATION each hook
// below performs is proved on the host anyway, by testing the free
// functions it delegates to directly: nezhaBoardDiagValue()
// (nezha_port.h/.cpp, host-tested by test_board_nezha_diag.py through
// nezha_board_diag_shim.cpp) and NezhaMotorPort's own
// wiredPort()/wiredSign()/configureWiring() (already host-portable,
// same test). What is NOT re-proved here is the wiring of THIS file's
// own thin one-line hooks into that computation -- see
// tests/host/test_board_seam_source_pin.py for the source-level pin
// that covers exactly that seam instead.
#include "board.h"

#if DIFFDRIVE_BOARD == DIFFDRIVE_BOARD_NEZHA

#include "nezha_port.h"

#ifndef DIFFDRIVE_HOST_BUILD
#include "pxt.h"
#endif

namespace diffDrive {

namespace {

// The board's own motor pair -- vevov wiring, VERIFIED 2026-08-20
// under AprilCam; see the history this comment used to carry inline in
// shims.cpp's Rig (git blame on this file finds it if needed again).
// A lazy Meyer's singleton, matching Rig's own lazy-construction shape
// (shims.cpp's ensure()): constructed on first use, not at static-init
// time, so it never races the static-initialization-order question
// defaultI2CBus() (platform/i2c_bus.h) was written to sidestep.
struct NezhaBoard {
  NezhaMotorPort left{1, -1};    // left = M1, mirrored
  NezhaMotorPort right{2, +1};   // right = M2
};

NezhaBoard& board() {
  static NezhaBoard instance;
  return instance;
}

}  // namespace

BoardMotors boardMotors() {
  NezhaBoard& b = board();
  return BoardMotors{b.left, b.right};
}

MotorWiring boardWiring(int side) {
  const NezhaBoard& b = board();
  const NezhaMotorPort& p = (side == 0) ? b.left : b.right;
  return MotorWiring{p.wiredPort(), p.wiredSign()};
}

void boardConfigureWiring(int side, uint8_t port, int8_t sign) {
  NezhaBoard& b = board();
  NezhaMotorPort& p = (side == 0) ? b.left : b.right;
  p.configureWiring(port, sign);
}

int boardDiagValue(int ordinal) {
  const NezhaBoard& b = board();
  return nezhaBoardDiagValue(b.left, b.right, ordinal);
}

// ---- hybrid actuation --------------------------------------------------
//
// The Nezha brick has no onboard speed loop, so none of this means
// anything here -- see board.h's own comment on the documented no-op/
// refusal shape. No NezhaBoard state backs these; there is nothing to
// remember.
WheelCommandTap* boardWheelCommandTap() { return nullptr; }
int boardOnboardMode() { return 0; }
bool boardSetOnboardMode(int) { return false; }
float boardOnboardFloor() { return 0.0f; }
bool boardSetOnboardFloor(float) { return false; }

#ifndef DIFFDRIVE_HOST_BUILD

// ---- fault-context emergency stop -----------------------------------
//
// Moved verbatim from nezha_port.cpp: the same bytes at the same
// address would land on a different board's MCU, so which board's
// definition gets linked in is now decided by DIFFDRIVE_BOARD.
// nezha_port.cpp's fault handlers (diffdriveFaultReport(),
// HardFault_Handler, ...) are board-generic ARM/CODAL plumbing and
// stay there, calling this function through the same `extern "C"`
// declaration they always have -- only WHICH board's frame answers
// that call moves with the board
// selection.
//
// Writes "run at 0" to BOTH motor ports over I2C with no dependency on
// any object, fiber, scheduler or kernel state -- so it is callable
// from a fault handler, where none of those can be trusted. See
// nezha_port.cpp's own (unmoved) comment on why a plain reboot is not
// sufficient on its own, and vfp_guard.h's discipline this call
// deliberately does NOT need (it never yields).
extern "C" void diffdrive_emergency_motor_stop() {
  // Same frame shape as NezhaMotorPort::writeFrame(), inlined so this
  // needs no instance: {0xFF, 0xF9, port, arg, reg, val, 0xF5, 0x00}.
  for (uint8_t port = 1; port <= 2; ++port) {
    uint8_t frame[8] = {0xFF, 0xF9, port, NezhaMotorPort::kDirCw,
                        NezhaMotorPort::kRegMotorRun, 0x00, 0xF5, 0x00};
#if MICROBIT_CODAL
    uBit.i2c.write(NezhaMotorPort::kAddress << 1, frame, 8);
#else
    uBit.i2c.write(NezhaMotorPort::kAddress << 1,
                   reinterpret_cast<char*>(frame), 8);
#endif
  }
}

#endif  // DIFFDRIVE_HOST_BUILD

}  // namespace diffDrive

#endif  // DIFFDRIVE_BOARD == DIFFDRIVE_BOARD_NEZHA
