// board_cutebot.cpp -- the Cutebot Pro half of the board-composition
// seam (board.h), compiled only when DIFFDRIVE_BOARD selects
// DIFFDRIVE_BOARD_CUTEBOT_PRO. Mirrors board_nezha.cpp's own shape
// exactly: the concrete two-port construction, the `configure motor`
// wiring hooks, the diag-ordinal readback, and the fault-context
// emergency-stop frame.
//
// SCOPE: the raw-PWM path (CutebotMotorPort/CutebotDevice,
// cutebot_port.h/.cpp) AND the `0x80` onboard speed-loop path and the
// WheelCommandTap/actuation-policy hybrid (design doc's own
// hybrid-actuation section) -- this file's CutebotBoard singleton also
// owns a CutebotTapAdapter (cutebot_port.h) bound to the SAME
// device/port pair, installed on MotionEngine through
// boardWheelCommandTap() below. Nothing about the two ports' own
// construction changed; the tap adapter sits beside them, using
// CutebotDevice's own API (stageDuty()'s coalescing point) exactly as
// it always worked.
//
// Host-compilability note, same shape as board_nezha.cpp's own: this
// file reaches `pxt.h` ONLY inside `#ifndef DIFFDRIVE_HOST_BUILD`, for
// the fault-context frame's real `uBit.i2c.write()`. Nothing else here
// is host-linkable as written: `CutebotBoard` constructs its
// `CutebotDevice` through the DEFAULT bus argument
// (`defaultI2CBus()`), which is defined only by the target's
// `platform/microbit_i2c_bus.cpp` -- there is no host definition, by
// design (`i2c_bus.h`'s own comment). This translation unit stays
// hex-checkpoint-only in its entirety, the same as board_nezha.cpp.
// The Cutebot-specific COMPUTATION each hook below performs is proved
// on the host anyway, via cutebot_port.h/.cpp directly
// (tests/host/test_cutebot_port.py) and via cutebotBoardDiagValue()
// (tests/host/test_board_nezha_diag.py's sibling for this board, where
// one exists). What is NOT re-proved here is the wiring of THIS file's
// own thin one-line hooks into that computation -- see
// tests/host/test_board_cutebot_source_pin.py for the source-level pin
// that covers exactly that seam instead.
#include "board.h"

#if DIFFDRIVE_BOARD == DIFFDRIVE_BOARD_CUTEBOT_PRO

#include "cutebot_port.h"

#ifndef DIFFDRIVE_HOST_BUILD
#include "pxt.h"
#endif

namespace diffDrive {

namespace {

// The board's own motor pair. A lazy Meyer's singleton, matching
// board_nezha.cpp's NezhaBoard shape: constructed on first use, not at
// static-init time. left/right sign defaults are +1/+1 -- UNVERIFIED
// against a real board (no Cutebot has been bench-wired yet, per
// the design doc's own S10.2, still an open "which board" question);
// a real mount's sign bake is future bring-up work.
struct CutebotBoard {
  CutebotDevice device;
  CutebotMotorPort left{device, 0, +1};   // side 0 = left
  CutebotMotorPort right{device, 1, +1};  // side 1 = right
  // The hybrid-actuation tap, bound to this SAME device/port pair --
  // see cutebot_port.h's own comment for why the sign conversion
  // happens here rather than in CutebotDevice itself.
  CutebotTapAdapter tap{device, left, right};
};

CutebotBoard& board() {
  static CutebotBoard instance;
  return instance;
}

}  // namespace

BoardMotors boardMotors() {
  CutebotBoard& b = board();
  return BoardMotors{b.left, b.right};
}

MotorWiring boardWiring(int side) {
  const CutebotBoard& b = board();
  const CutebotMotorPort& p = (side == 0) ? b.left : b.right;
  return MotorWiring{p.wiredPort(), p.wiredSign()};
}

void boardConfigureWiring(int side, uint8_t port, int8_t sign) {
  CutebotBoard& b = board();
  CutebotMotorPort& p = (side == 0) ? b.left : b.right;
  // The WiringResult this returns (kOk / kUnimplemented for a
  // port-changing request -- see cutebot_port.h's own comment) is
  // intentionally discarded here: this hook's signature is fixed by
  // board.h (a contract shared with board_nezha.cpp's
  // identical hook), and nothing today reads a configureMotor()
  // outcome back out through the wire (shims.cpp's configureMotor() is
  // itself a void MakeCode block shim -- see this ticket's own
  // completion notes for the full reasoning). test_cutebot_port.py
  // exercises the refusal directly against CutebotMotorPort::
  // configureWiring() instead of through this pass-through.
  p.configureWiring(port, sign);
}

int boardDiagValue(int ordinal) {
  const CutebotBoard& b = board();
  return cutebotBoardDiagValue(b.left, b.right, ordinal);
}

// ---- hybrid actuation --------------------------------------------------

WheelCommandTap* boardWheelCommandTap() { return &board().tap; }

int boardOnboardMode() { return board().device.onboardMode(); }
bool boardSetOnboardMode(int mode) { return board().device.setOnboardMode(mode); }
float boardOnboardFloor() { return board().device.onboardFloor(); }
bool boardSetOnboardFloor(float floor) {
  return board().device.setOnboardFloor(floor);
}

#ifndef DIFFDRIVE_HOST_BUILD

// ---- fault-context emergency stop -----------------------------------
//
// ONE frame zeroes BOTH wheels -- unlike Nezha's two independently
// addressed brick ports, the Cutebot Pro is a single 0x10 slave, so
// the v2 "wheel = both" selector (SOURCE READING,
// the design doc's own S1.2) covers the whole board in
// one write, with no dependency on any object, fiber, scheduler or
// kernel state -- see nezha_port.cpp's own comment for why that
// matters from a fault handler, and board_nezha.cpp's own definition
// of the same extern "C" function for the Nezha frame this one
// replaces when DIFFDRIVE_BOARD selects Cutebot instead.
extern "C" void diffdrive_emergency_motor_stop() {
  // Same frame shape as CutebotDevice::writeWheelFrame(), inlined so
  // this needs no instance: {0xFF, 0xF9, cmd, len, wheel=both, absL=0,
  // absR=0, dirbits=0}.
  uint8_t frame[8] = {0xFF, 0xF9, CutebotDevice::kCmdWheel, 0x04,
                      CutebotDevice::kWheelBoth, 0x00, 0x00, 0x00};
#if MICROBIT_CODAL
  uBit.i2c.write(CutebotDevice::kAddress << 1, frame, 8);
#else
  uBit.i2c.write(CutebotDevice::kAddress << 1,
                 reinterpret_cast<char*>(frame), 8);
#endif
}

#endif  // DIFFDRIVE_HOST_BUILD

}  // namespace diffDrive

#endif  // DIFFDRIVE_BOARD == DIFFDRIVE_BOARD_CUTEBOT_PRO
