// board.h -- the compile-time board-composition seam.
//
// One literal, DIFFDRIVE_BOARD, decides which physical board this
// build composes against -- selected once, at compile time, the same
// way kChannel/kProfile/firmware_bake.motors already are
// (tools/make_deploy.py's own comment on this seam explains why a
// compile-time bake beats runtime 0x10 autodetection: two open issues
// on first-I2C-command wedges, and this fleet's identity-comes-from-
// something-baked rule). Absent a `geometry.firmware_bake.board` key
// (every fleet robot today), it defaults to Nezha, so an existing
// build is unaffected by this file's mere existence.
//
// This header is deliberately host-portable: the macro literal and the
// declarations below reach nothing but core/diffdrive.h (the Motor
// interface) and core/motor_wiring.h (the pure wiring decision), both
// already host-compiled. The TARGET-bound half of each board's own
// composition -- constructing real ports, writing the fault-context
// emergency-stop frame over real I2C -- lives in board_nezha.cpp /
// board_cutebot.cpp, not here; see those files' own header comments
// for which parts of THEM reach pxt.h.
#pragma once

#include "../core/diffdrive.h"
#include "../core/motor_wiring.h"
#include "../motion/wheel_command_tap.h"

#define DIFFDRIVE_BOARD_NEZHA 1
#define DIFFDRIVE_BOARD_CUTEBOT_PRO 2

#ifndef DIFFDRIVE_BOARD
#define DIFFDRIVE_BOARD DIFFDRIVE_BOARD_NEZHA
#endif

namespace diffDrive {

// The composed board's motor pair. Owned by the board's own singleton
// (board_nezha.cpp's NezhaBoard, board_cutebot.cpp's own equivalent),
// constructed lazily on first call, living for the process's whole
// lifetime -- the same lazy-singleton shape shims.cpp's own Rig/
// ensure() already uses. Rig::left/right (shims.cpp) bind to these
// references once, at Rig's own construction, in place of declaring
// concrete NezhaMotorPort members directly.
struct BoardMotors {
  DiffDrive::Motor& left;
  DiffDrive::Motor& right;
};

// Constructs (on first call) and returns this build's motor pair.
BoardMotors boardMotors();

// Which physical port/sign one side of the composed board is
// currently wired to, and applying a change to it -- the two halves
// `configure motor` needs (shims.cpp's configureMotor()) that only the
// composed board's own concrete port type can answer, since
// DiffDrive::Motor itself carries no notion of "port". side: 0 = left,
// 1 = right. The wiring DECISION (is this a swap, does the pair end up
// sharing a port) stays entirely in motor_wiring.h's
// applyWiringRequest() -- these two hooks only let a board-agnostic
// caller read and apply the result against whichever concrete port
// type this build composed.
MotorWiring boardWiring(int side);
void boardConfigureWiring(int side, uint8_t port, int8_t sign);

// diagValue() ordinals 21/22/23/24/27/35-40 (shims.cpp) -- Nezha-only
// counters and wiring readbacks that used to read `ensure().left`/
// `.right` directly as a NezhaMotorPort. A board with no meaning for a
// given ordinal answers 0, matching diagValue()'s own existing
// default; see board_nezha.cpp's own doc comment for the Nezha
// mapping, factored into nezha_port.h's host-testable
// nezhaBoardDiagValue() so it is provable without a live board.
int boardDiagValue(int ordinal);

// The role string HELLO's banner reports (comms/protocol.cpp's kRole)
// -- distinct from kDrivetrain, which never varies by board. Not a
// runtime hook like the three above: protocol.cpp selects the literal
// directly via `#if DIFFDRIVE_BOARD == ...`, so kRole stays the plain
// `constexpr const char*` its existing runtime-override mechanism
// already seeds roleBuf_ from -- this header only carries the macro
// that selection is made against.

// The optional WheelCommandTap this board installs on MotionEngine --
// shims.cpp's ensure() calls this once, right after constructing Rig,
// and forwards the result straight to `engine.setWheelCommandTap()`. A
// board with no use for the shaped setpoint (Nezha, forever) returns
// nullptr, which MotionEngine already treats as "no tap installed" --
// byte-identical to before this hook existed. Constructed lazily
// alongside boardMotors()'s own singleton; see each board's own
// definition.
WheelCommandTap* boardWheelCommandTap();

// Hybrid actuation config (comms/config_fields.h ordinals 43/44): the
// mode (0 off / 1 threshold-with-hysteresis / 2 plateau-only) and floor
// [mm/s] CutebotActuationPolicy reads each cycle. Meaningless on a
// board with no onboard loop -- board_nezha.cpp's
// own implementation reports mode 0 / floor 0 on GET and refuses every
// SET (returns false), which shims.cpp's config accessors treat as a
// silent no-op, the same "ignored, not erred" shape every other
// out-of-range wire SET in this codebase gets. `SET onboard_pid 0`
// still always ACKS on every board -- refusing the WIRE command is not
// the same as refusing to store a value nothing on this board reads.
int boardOnboardMode();
bool boardSetOnboardMode(int mode);
float boardOnboardFloor();
bool boardSetOnboardFloor(float floor);

}  // namespace diffDrive
