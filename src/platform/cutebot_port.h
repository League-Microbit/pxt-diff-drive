// cutebot_port.h -- CutebotDevice + CutebotMotorPort: the ELECFREAKS
// Cutebot Pro's raw-PWM actuation path implemented against
// DiffDrive::Motor, for the MakeCode target.
//
// SOURCE READING, not MEASURED (see this repo's own measurement-
// citations rule): every wire-protocol claim below is transcribed
// from the Cutebot Pro design doc SS1.1-1.3, itself read from
// ELECFREAKS' `pxt-Cutebot-Pro` extension (its top-level module plus
// `v2.ts`, fetched 2026-09-21, NOT vendored). Nothing here has run on
// a Cutebot Pro yet.
//
// WHY ONE DEVICE, TWO PORTS. The Cutebot Pro is ONE I2C slave at 0x10
// with no per-motor addressing -- its `0x10` command takes BOTH
// wheels' duties in a single frame (`wheel(0 L/1 R/2 both), abs(L)%,
// abs(R)%, dirbits`). `CutebotDevice` is that slave's session state,
// shared by both `CutebotMotorPort`s; each port stages its own duty
// via `CutebotDevice::stageDuty()`, which ships the coalesced frame
// the moment BOTH sides have staged since the last ship -- so
// whichever wheel's `tick()` completes the pair each kernel cycle is
// the one that actually writes the bus, and the frame count per cycle
// is exactly one regardless of which side the kernel happens to drive
// first (`core/diffdrive.cpp`'s own `left_.tick()` then `right_.tick()`
// order is NOT relied on here).
//
// WIRE PROTOCOL (v2 frames, `FF F9 <cmd> <len> <params...>`, no
// trailing checksum -- distinct from the Nezha brick's 8-byte
// fixed-shape frame and from the Cutebot's OWN v1 frame shape):
//
//   cmd 0x10 (len 4): wheel(0 L/1 R/2 both), abs(L)%, abs(R)%,
//     dirbits(b0 = L reverse, b1 = R reverse)      -- raw PWM
//   cmd 0xA0 (len 1): [3] or [4]                    -- SELECTS that
//     wheel's accumulated-degrees register for the NEXT bare read
//   (a bare 4-byte read after the 0xA0 select above): Int32LE degrees
//   cmd 0x50 (len 1): motor(0 L/1 R)                -- zero that
//     wheel's HARDWARE encoder register
//
// Revision probe (the extension's top-level `readHardVersion()`, NOT
// a v2 frame): write the fixed 7 bytes `99 15 01 00 00 00 88`, then
// read one byte -- `1` means hardware v1, anything else v2. Cached for
// the session at the first `begin()` either wheel calls
// (`CutebotDevice::ensureProbed()`). This ticket assumes a v2 board
// throughout (`isV2()` is recorded but not yet branched on); a v1
// board is a follow-on ticket's problem.
//
// ENCODER UNITS. The wire reports whole DEGREES (1 LSB = 1 deg); the
// kernel's own unit is 0.1 deg per count (`core/diffdrive.h`), so
// `degrees x 10 = counts` -- the SAME ratio `NezhaMotorPort` uses,
// chosen so nothing above the port needs to know which brick it is
// talking to.
//
// MINIMAL SHAPING, DELIBERATELY. `NezhaMotorPort`'s reversal dwell,
// sigma-delta quantizer and write throttle each guard a MEASURED
// Nezha hardware failure (see that file's own header). None of them
// is known to apply here -- the design doc's own S3.A
// says so explicitly ("the new port starts with none of them and
// earns each one on the bench") -- so `setDuty()`/`tick()` below do
// the plainest possible thing: clamp, round to a percent, ship. The
// same applies to `wedged()`/`wedgeSuspect()`: no streak detector
// exists yet, so both always answer false rather than encode an
// unverified guess at what a Cutebot wedge even looks like.
//
// FIBER-YIELD SAFETY. The extension's own `i2cCommandSend()` follows
// every v2 write with a 1 ms busy-wait (S1.2). On the target that must
// be a guarded fiber yield, never a spin or a raw `fiber_sleep()` (see
// this repo's own fiber-yield-safety rule) -- `CutebotDevice`'s
// low-level `writeV2Frame()` is the ONE place that wait happens, via
// `diffDrive::vfpSafeSleep()` (`vfp_guard.h`). The v1-style revision
// probe is NOT a v2 frame and does not go through `writeV2Frame()`, so
// it gets no such wait -- UNVERIFIED whether a real board needs one
// there too; nothing in the source reading says so.
//
// HOST-COMPILABLE, ENTIRELY. Unlike `nezha_port.h/.cpp`, this file has
// no CODAL-facing half at all: the fault-context
// `diffdrive_emergency_motor_stop()` frame for a Cutebot board lives
// in `board_cutebot.cpp` instead (mirroring `board_nezha.cpp`'s own
// split), since it needs no `CutebotDevice` instance and must build
// with no object, fiber, scheduler or kernel state in reach. Nothing
// here reaches `pxt.h`, directly or transitively -- `i2c_bus.h`'s
// `defaultI2CBus()` is a DECLARATION only; its target-only definition
// is never referenced unless a caller actually omits the bus argument,
// which no host test does.
//
// HYBRID ACTUATION. CutebotDevice also owns the `0x80` onboard-speed-
// loop path and the per-cycle choice between it and the `0x10` PWM
// path above -- see CutebotActuationPolicy (cutebot_actuation_policy.h)
// for the pure decision itself, and CutebotTapAdapter (below) for how a
// WheelCommandTap notification (motion/wheel_command_tap.h) reaches
// this device with the correct WIRE sign already applied. `stageDuty()`
// (this file's own "coalescing point") runs that decision once both
// wheels have staged, and ships EXACTLY ONE frame: the coalesced PWM
// frame, or one `0x80` setpoint frame -- never both, except the one
// documented exception in serviceCycle()'s own comment (a genuine stop
// while the onboard loop held the wheels).
#pragma once

#include <cstdint>

#include "i2c_bus.h"
#include "cutebot_actuation_policy.h"
#include "../core/diffdrive.h"
#include "../motion/wheel_command_tap.h"

namespace diffDrive {

// Mirrors comms::Wire::Result's refusal-code TAXONOMY
// (src/comms/wire_handler.h) without including it. This package's own
// layer-map document is explicit that "Hardware ports... know
// I2C/CODAL, nothing about blocks or the wire" -- platform/ may not
// depend on comms/, so `CutebotMotorPort::configureWiring()` cannot
// literally return a `Wire::Result` without inverting that dependency
// direction. This enum is the platform-local echo of the same two
// codes that matter here, chosen so a future caller that DOES sit
// above both layers (there is none today -- see this header's own
// note on `configureWiring()` below) can map it 1:1 with no semantic
// translation.
//
// STAKEHOLDER-REVIEWABLE (an explicitly open design question in the
// design doc): `kUnimplemented` for a port-changing request, not
// `kBadArg`/`kRange`. A port change is not malformed input (the
// requested port number is a perfectly valid Nezha-style 1..4 value in
// the wire's own vocabulary) and not out of range (ports 1-4 are all
// individually valid elsewhere) -- it fails only because THIS board
// has nothing a second port could mean. `kUnimplemented` is the
// taxonomy's own code for "well-formed request, not supported here",
// which is exactly that.
enum class WiringResult : uint8_t {
  kOk,
  kUnimplemented,
};

class CutebotMotorPort;  // forward: CutebotTapAdapter below only needs
                         // a reference to it (wiredSign()), defined in
                         // this file further down.

class CutebotDevice {
 public:
  // The default bus preserves the zero-argument construction shape
  // `board_cutebot.cpp`'s singleton uses on the target; host tests
  // inject a simulated bus explicitly.
  explicit CutebotDevice(I2CBus& bus = defaultI2CBus()) : bus_(bus) {}

  // Revision probe, run once per session and cached -- see this file's
  // header comment. Returns false (and leaves the cache unprimed, so a
  // later retry is possible) only if the bus itself failed; the v1/v2
  // RESULT is always available afterward via isV2().
  bool ensureProbed();
  bool isV2() const { return isV2_; }

  // side: 0 = left, 1 = right. Stages this side's WIRE-signed duty
  // percent (already carrying fwdSign_ -- CutebotDevice knows nothing
  // about sign conventions, only bytes). Ships the coalesced 0x10
  // frame the moment BOTH sides have staged since the last ship, then
  // clears both staged flags -- see this file's header comment for why
  // this makes "the second wheel's tick() ships" true regardless of
  // call order.
  void stageDuty(int side, int8_t wireValue);

  // Immediate, unstaged zero for ONE side only (wheel selector 0 or 1,
  // never "both") -- CutebotMotorPort::emergencyStop()'s bus call.
  // Does not touch the other side's staged duty or the staged flags.
  bool writeSingleWheelZero(int side);

  // Split-phase encoder, mirroring NezhaMotorPort's requestSample()/
  // collect() split: selectDegrees() writes the 0xA0 query (and its
  // own settle wait); readSelectedDegrees() is the bare follow-up read.
  bool selectDegrees(int side);
  bool readSelectedDegrees(int32_t* degreesOut);

  // cmd 0x50: zero a wheel's HARDWARE encoder register. NOT called by
  // CutebotMotorPort::rebaseline() (software-only offset, exactly like
  // NezhaMotorPort) -- exposed for parity with the design doc's own
  // description of what the device can do, and for any future caller
  // that wants a real device zero.
  bool hardwareClearEncoder(int side);

  // ---- hybrid actuation ----
  //
  // CutebotTapAdapter's own onDrive()/onNeutral() forwards, already
  // WIRE-signed (see that class's own comment for why the sign
  // conversion happens there, not here). Recorded, not acted on
  // immediately -- serviceCycle() (private, below) reads the latest of
  // these the next time both wheels have staged, which is this
  // project's own definition of "once per kernel cycle" (stageDuty()'s
  // both-sides-staged gate).
  void updateTapDrive(float left, float right,
                     VelocityShaper::Phase phase);
  void updateTapNeutral();

  // `onboard_pid`/`onboard_floor` (comms/config_fields.h ordinals
  // 43/44), read and written through board_cutebot.cpp's thin
  // board.h hooks. Mode is validated to {0,1,2}; an out-of-range SET is
  // refused (returns false) and leaves the stored mode untouched --
  // the same "refused, not silently coerced" shape
  // CutebotMotorPort::configureWiring() uses for its own out-of-range
  // inputs, except a refused SIGN there is a free no-op while an
  // out-of-range MODE here has no sensible "closest legal value" to
  // fall back to, so refusing outright is the honest answer. Floor
  // must be strictly positive (a zero or negative floor would make
  // every nonzero setpoint "at or above the floor", defeating the
  // eligibility gate's whole point).
  int onboardMode() const { return onboardMode_; }
  bool setOnboardMode(int mode);
  float onboardFloor() const { return onboardFloor_; }
  bool setOnboardFloor(float floor);

  // Public so board_cutebot.cpp's fault-context emergency-stop frame
  // can build the SAME wire frame with no CutebotDevice instance in
  // reach -- see that file's own comment for why that path cannot use
  // object state. kAddress/kCmdWheel happen to share the numeric value
  // 0x10 -- one is the I2C slave address, the other the PWM command
  // opcode; the coincidence is the extension's, not a copy-paste bug.
  static constexpr uint8_t kAddress = 0x10;
  static constexpr uint8_t kCmdWheel = 0x10;
  static constexpr uint8_t kWheelLeft = 0;
  static constexpr uint8_t kWheelRight = 1;
  static constexpr uint8_t kWheelBoth = 2;

 private:
  static constexpr uint8_t kCmdEncoder = 0xA0;
  static constexpr uint8_t kCmdClear = 0x50;
  static constexpr uint8_t kCmdOnboard = 0x80;
  static constexpr uint8_t kSelDegreesLeft = 3;
  static constexpr uint8_t kSelDegreesRight = 4;

  // The one low-level write primitive: builds `FF F9 <cmd> <len>
  // <params>`, writes it, and performs the extension's own 1 ms
  // post-write wait (guarded -- see this file's header comment).
  // paramLen is at most 5 (cmd 0x80's own params, the largest frame
  // this device ever sends).
  bool writeV2Frame(uint8_t cmd, const uint8_t* params, uint8_t paramLen);
  bool writeWheelFrame(uint8_t wheelSel, uint8_t absL, uint8_t absR,
                       uint8_t dirbits);
  void shipFrame();

  // cmd 0x80: `Lh Ll Rh Rl dirbits`, mm/s magnitude per wheel (design
  // doc S1.2), ALWAYS both wheels -- there is no per-wheel selector the
  // way 0x10's `wheel` byte has one. Deliberately UNCLAMPED here (see
  // sim_cutebot_bus.h's own header comment on why the 200..500 mm/s
  // clamp lives on the receiving end of this frame, not the sending
  // end): this ships exactly whatever CutebotActuationPolicy's tapped
  // setpoint was, so a policy bug that violates its own floor is
  // visible in what the simulated wheel actually does, not silently
  // absorbed here. Magnitude is bounded to fit 16 bits purely as an
  // overflow guard, unrelated to the 200/500 mm/s clamp.
  bool writeOnboardFrame(float left, float right);

  // The per-cycle decision (design doc's hybrid-actuation section):
  // called from stageDuty() the moment both wheels have staged, exactly
  // where shipFrame() used to be called unconditionally. Runs
  // CutebotActuationPolicy::decide() once, ships the ONE frame it
  // chooses, and -- the one documented two-frame exception -- also
  // ships an 0x80 zero-both frame on the specific transition from
  // "onboard was engaged" to "a neutral/stop tick arrived", per the
  // ticket's own handoff-bookkeeping requirement: "on neutral/stop
  // while onboard is engaged, send the 0x80 zero AND the 0x10 zero --
  // which one really stops a wheel under onboard control is a
  // sprint-041 bench probe, and both are cheap." Every other
  // engaged->disengaged transition (hysteresis release, the plateau's
  // first brake tick, an eligibility drop) ships ONLY the frame
  // decide() chose -- "the device resumes shipping the kernel's duty"
  // (this ticket's own completion notes) needs no second frame, since
  // shipFrame() below already sends whatever the kernel's own PI loop
  // is currently asking for, not a zero.
  void serviceCycle();

  I2CBus& bus_;  // the wire; never owned

  bool probed_ = false;
  bool isV2_ = true;

  int8_t stagedValue_[2] = {0, 0};
  bool staged_[2] = {false, false};

  // Hybrid actuation state.
  int onboardMode_ = 0;            // GET/SET onboard_pid, {0,1,2}
  float onboardFloor_ = 200.0f;    // GET/SET onboard_floor [mm/s]
  CutebotActuationPolicy::PolicyState policyState_{};

  // The latest WheelCommandTap sample, already wire-signed (see
  // CutebotTapAdapter below). `tapNeutral_` starts true: with no tap
  // installed (every Nezha build, and a Cutebot build before its first
  // drive tick) this device must behave exactly as the raw-PWM path
  // always did -- decide() forces PWM on a neutral tick regardless of mode, so an
  // un-notified device is indistinguishable from mode 0.
  float tapLeft_ = 0.0f;
  float tapRight_ = 0.0f;
  VelocityShaper::Phase tapPhase_ = VelocityShaper::Phase::kAccel;
  bool tapNeutral_ = true;
};

// Forwards WheelCommandTap notifications into a CutebotDevice,
// converting MotionEngine's caller-space (left, right) mm/s into
// WIRE-signed values using each side's own fwdSign_ -- the SAME sign
// CutebotMotorPort::tick() applies to its own staged PWM duty. This is
// the one place both wheels' signs are available together: a
// WheelCommandTap notification fires once per tick for BOTH wheels,
// not once per port, and CutebotDevice itself stays free of per-wheel
// sign knowledge (stageDuty() already receives pre-signed wire values
// from each port, and this adapter keeps that convention intact for
// the tap's own numbers).
class CutebotTapAdapter final : public WheelCommandTap {
 public:
  CutebotTapAdapter(CutebotDevice& device, const CutebotMotorPort& left,
                    const CutebotMotorPort& right)
      : device_(device), left_(left), right_(right) {}

  void onDrive(float left, float right,
              VelocityShaper::Phase phase) override;
  void onNeutral() override { device_.updateTapNeutral(); }

 private:
  CutebotDevice& device_;
  const CutebotMotorPort& left_;
  const CutebotMotorPort& right_;
};

class CutebotMotorPort final : public DiffDrive::Motor {
 public:
  // device: the shared 0x10 slave both sides bind to. side: 0 = left,
  // 1 = right -- fixed for this instance's lifetime except through
  // configureWiring()'s sign-only path below. fwdSign: +1/-1, applied
  // to duty AND encoder, exactly like NezhaMotorPort.
  CutebotMotorPort(CutebotDevice& device, int side, int8_t fwdSign)
      : device_(device), side_(side), fwdSign_(fwdSign) {}

  // ---- DiffDrive::Motor ----
  void begin() override;
  void requestSample() override;
  void setDuty(float duty) override;      // [-1, 1] staged, caller-space
  void emergencyStop() override;          // zero NOW, unstaged
  void tick(uint64_t now) override;     // [us] ship staged + collect
  float position() const override;        // [counts]
  float velocity() const override;        // [counts/s]
  float appliedDuty() const override;     // [-1, 1] caller-space, last staged
  bool connected() const override { return connected_; }
  uint64_t sampleTime() const override { return sampleTime_; }
  void rebaseline() override;             // software re-anchor, no bus
  bool wedged() const override { return false; }        // see header: no
  bool wedgeSuspect() const override { return false; }  // detector yet

  // Re-point this side's SIGN at RUN time; a port-changing request is
  // refused (see this file's own WiringResult comment above). Mirrors
  // NezhaMotorPort::configureWiring()'s signature in spirit, not in
  // return type -- board_cutebot.cpp's boardConfigureWiring() (a hook
  // whose own signature is a cross-board contract this file does not
  // own, fixed as void) calls this and discards the result; this
  // method itself is the place a future wire-level caller would read
  // the refusal from.
  WiringResult configureWiring(uint8_t port, int8_t sign);

  uint8_t wiredPort() const { return static_cast<uint8_t>(side_ + 1); }
  int8_t wiredSign() const { return fwdSign_; }

 private:
  void collect(uint64_t now);  // [us]

  CutebotDevice& device_;
  int side_;          // 0 = left, 1 = right
  int8_t fwdSign_;    // [+1/-1]

  float stagedDuty_ = 0.0f;  // [-1,1] caller-space
  int lastApplied_ = 0;      // [pct] caller-space, last shipped

  int32_t encOffset_ = 0;  // [counts] software rebaseline
  int32_t lastRaw_ = 0;    // [counts] last successfully read raw value,
                           // BEFORE encOffset_ -- what rebaseline()
                           // re-anchors to, mirroring NezhaMotorPort's
                           // glitchArmor_.lastGoodRaw()
  float lastPosition_ = 0.0f;   // [counts]
  float velocity_ = 0.0f;       // [counts/s]
  uint64_t sampleTime_ = 0;   // [us] last SUCCESSFUL collect
  uint64_t lastTick_ = 0;     // [us]
  bool hasLastTick_ = false;
  bool connected_ = false;
};

// board_cutebot.cpp's boardDiagValue() hook -- ordinals 35-38 (the live
// wiring readback) only. No glitch-armor or rebaseline counters exist
// yet (see this file's own "minimal shaping" note), so every other
// board-generic ordinal falls through to diagValue()'s own default of
// 0, exactly as it does for a board with no meaning for it.
int cutebotBoardDiagValue(const CutebotMotorPort& left,
                          const CutebotMotorPort& right, int ordinal);

}  // namespace diffDrive
