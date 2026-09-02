// wire_adapter.cpp -- see wire_adapter.h for the class contract.
#include "wire_adapter.h"

#include <cmath>
#include <cstring>

namespace diffDrive {

// ---- shims.cpp entry points: plain same-namespace forward
// declarations (that file has no header of its own), kept
// signature-compatible with shims.cpp's real definitions -- same
// convention protocol.cpp uses to reach the same file. `rotationRad`
// (engineMoveX()) and `omegaRad` (engineMoveV()) arrive ALREADY
// converted from the wire's milliradian integers -- see mradToRad()
// below. `cruise`/`speed`'s wire "0 means configured default" sentinel
// is resolved by the onXxx() handlers below, via
// engineDefaultCruiseMmS(), BEFORE any of these functions is ever
// called -- none of them ever sees the sentinel itself. engineGoToW()
// selects its own PoseSource (OtosPort when connected(), else the
// encoder-odometry fallback -- motion-api.md S3.6) and always
// dispatches; its bool return -- "was a live pose available" -- is now
// unconditionally true, kept for contract stability.
void stopAll();
void estopAll();
void setWheelsTimed(int left, int right, uint32_t durationMs);
void setKernelValue(int field, int value);
int getConfigValue(int field);
int diagValue(int what);

void engineWheelsX(float left, float right, float cruise,
                   uint32_t timeoutMs);
void engineMoveX(float distance, float rotationRad, float cruise,
                 uint32_t timeoutMs);
float engineDefaultCruiseMmS();
// SUC-003: engineADecelMmS2() (MotionEngine::aDecelMmS2()'s own wire-
// shaped forward) lets the onXxx() handlers below decide, per call,
// whether their own `cruise == 0` sentinel resolves from the flat
// engineDefaultCruiseMmS() above or from engineDefaultCruiseForDistanceMmS()'s
// distance-aware resolve -- see resolveDefaultCruiseMmS() below.
float engineADecelMmS2();
float engineDefaultCruiseForDistanceMmS(float distanceMm);
// SUC-003: onMoveX()'s own D input -- a pure pivot (distance == 0)
// still has a real wheel-travel distance; see resolveDefaultCruiseMmS()
// below and this function's own comment (shims.cpp).
float engineDominantAxisTravelMm(float distanceMm, float rotationRad);

void engineMoveV(float vx, float omegaRad, uint32_t durationMs);
void engineGoToR(float x, float y, float speed, float arrive,
                 uint32_t timeoutMs);
bool engineGoToW(float x, float y, float speed, float arrive,
                 uint32_t timeoutMs);
// SUC-003: onGoToW()'s own D input -- the TRUE body-frame chord from
// the robot's CURRENT pose to this call's world-frame target, not
// hypot(x, y) of the target alone (see this function's own comment,
// shims.cpp, for why that would be wrong).
float engineGoToWChordMm(float worldX, float worldY);

// ---- shims.cpp entry points (sprint 004 ticket 004) ---------------------
// buildSnapshot()'s own five reads, reaching live pose/OTOS/wheel-speed
// state -- every one of these already exists in shims.cpp today (this
// ticket adds ZERO new entry points there), same same-package
// forward-declaration convention as every block above.
//
// THREE HAZARDS, each with real debugging history on this project:
//
//   1. poseX()/poseY()/poseHeading() each call odomUpdate() internally
//      and therefore MUTATE odometry -- this is LOAD-BEARING, not an
//      accident to "optimize" into one cached read: between moves,
//      nothing else advances odometry, so the 50 ms telemetry tick is
//      what keeps pose current while the robot sits idle. Collapsing
//      three calls into one would silently stop that.
//   2. otosGet(0)/otosGet(1) are 0.1 mm (divide by 10 for ox/oy);
//      otosGet(2) is ALREADY centidegrees (oh) -- do not also divide it.
//   3. otosGet() reads a CACHE. The protocol fiber must NEVER trigger a
//      fresh sensor sample (this project's own OtosPort blocking-read
//      primitive, a DIFFERENT, deliberately-not-named-here entry point
//      from this ordinal-based cache accessor) -- an I2C transaction
//      interposed in the Nezha encoder's select->read window destroys
//      the sample (Phase F). See
//      tests/host/test_wire_telemetry_projection.py's own source-text
//      check for the enforcement: that blocking-read entry point's own
//      name must appear NOWHERE in this file, comments included -- a
//      one-careless-line-away, catastrophic, SILENT failure mode.
int poseX();          // [mm] MUTATES odometry -- see hazard 1 above
int poseY();          // [mm] MUTATES odometry -- see hazard 1 above
int poseHeading();    // [cdeg] MUTATES odometry -- see hazard 1 above
int otosGet(int what);  // CACHE ONLY -- see hazard 3 above; never the
                         // blocking-read entry point
int wheelSpeed(int which);  // [mm/s]; which: 0 = left, 1 = right

// ---- shims.cpp entry point (sprint 005 ticket 004: the motion-
// completion signal, closing wire-motion-completion-signal.md/R-23) ----
// The ONE genuinely new read this ticket needs (sprint.md's own Design
// Rationale) -- true iff MotionEngine's own move-engine state is
// currently active, read-only, thin, matching engineWheelsX()'s own
// forward-declaration convention exactly. Used only by
// resolvePendingReason() (below) for the three goal-directed verbs
// (MOVE_X/GO_TO_R/GO_TO_W); WHEELS_V/WHEELS_X/MOVE_V never need it
// (they never make MotionEngine's move-engine state active in the
// first place -- wheelsV()/wheelsX() both call cancelMove() first,
// motion_engine.cpp). This class still holds no stored reference to
// MotionEngine/Rig -- see this file's own header comment.
bool engineMoveActive();

namespace {

// The `ConfigField` enum entries (`blocks/motion.ts`) mapped onto
// setKernelValue()/getConfigValue()'s existing field ordinals
// (shims.cpp) -- one wire NAME per field, replacing the old binary
// CONFIG/SET_FIELD/GET_CONFIG verbs' bare ordinal one-for-one.
// Declaration order matches ConfigField's own declaration order so a
// bare GET's dump reads in the same order a human reading `blocks/motion.ts`
// would expect. See each entry below for its own ordinal provenance.
struct FieldEntry {
  const char* name;  // wire key
  int ordinal;        // shims.cpp's setKernelValue()/getConfigValue() field
};

constexpr FieldEntry kFields[] = {
    {"max_duty", 0},           // ConfigField.MaxDuty
    {"full_duty_velocity", 1}, // ConfigField.FullDutyVelocity
    {"pid_kp", 2},             // ConfigField.Kp
    {"pid_ki", 3},             // ConfigField.Ki
    {"pid_i_max", 4},          // ConfigField.IMax
    {"accel_kaff", 5},         // ConfigField.Kaff
    {"pid_max", 6},            // ConfigField.PidMax
    {"twist_hold_gain", 7},    // ConfigField.TwistHoldGain
    {"speed_floor", 8},        // ConfigField.SpeedFloor
    {"pos_err_max", 9},        // ConfigField.PosErrMax
    {"stall_speed", 10},       // ConfigField.StallSpeed
    {"stall_demand", 11},      // ConfigField.StallDemand
    {"stall_window", 12},      // ConfigField.StallWindow
    {"lambda_enabled", 13},    // ConfigField.LambdaEnabled
    {"crawl_pulse", 14},       // ConfigField.CrawlPulse
    {"default_cruise", 15},    // ConfigField.DefaultCruise (sprint 007
                               // ticket 003, closing R-11/BLK-03/API-03
                               // -- see shims.cpp's engineDefaultCruiseMmS()/
                               // Rig::defaultCruiseMmS_ for the field this
                               // ordinal actually reaches).
    {"rotational_slip", 16},   // ConfigField.RotationalSlip (sprint 007
                               // ticket 005, closing R-14/API-06 -- see
                               // motion_engine.h's setRotationalSlip()/
                               // rotationalSlip_ for the validation and
                               // the load-bearing derivation comment on
                               // the 0.952 default).
    {"stall_clear", 17},       // ConfigField.StallClear (sprint 007
                               // ticket 001) -- a write-triggered action
                               // wearing a config-field's clothes; see
                               // shims.cpp's setKernelValue()/
                               // getConfigValue() case 17 and
                               // clearStall()'s own comment.
    {"pivot_overrun", 18},     // ConfigField.PivotOverrun (2026-08-29,
                               // OOP: per-wheel end-of-move overrun,
                               // mm, subtracted from every rotation
                               // target -- MotionEngine::
                               // setPivotOverrunMm()/pivotOverrunMm(),
                               // see motion_engine.h's own comment for
                               // the measurement).
    {"accel", 19},             // ConfigField.Accel -- [mm/s^2] thin
                               // forward to MotionEngine::
                               // setAAccelMmS2()/aAccelMmS2(); 0
                               // selects legacy mode (see that field's
                               // own comment, motion_engine.h).
    {"decel", 20},             // ConfigField.Decel -- [mm/s^2] thin
                               // forward to MotionEngine::
                               // setADecelMmS2()/aDecelMmS2(); 0
                               // selects legacy mode.
    {"v_max", 21},             // ConfigField.VMax -- [mm/s] thin
                               // forward to MotionEngine::
                               // setVMaxMmS()/vMaxMmS(), the ceiling
                               // defaultCruiseForDistance() never
                               // exceeds.
    {"brake_frac", 22},        // ConfigField.BrakeFrac -- [1] thin
                               // forward to MotionEngine::
                               // setBrakeFrac()/brakeFrac(), the share
                               // of a leg's own length
                               // defaultCruiseForDistance() allots to
                               // braking.
    {"dist_taper", 23},        // ConfigField.DistTaper -- [counts] thin
                               // forward to MotionEngine::
                               // setDistTaper()/distTaper(), previously
                               // reachable only from TypeScript.
    {"yaw_taper", 24},         // ConfigField.YawTaper -- [counts] thin
                               // forward to MotionEngine::
                               // setYawTaper()/yawTaper().
    {"dist_floor", 25},        // ConfigField.DistFloor -- [1] thin
                               // forward to MotionEngine::
                               // setDistFloor()/distFloor().
    {"turn_floor", 26},        // ConfigField.TurnFloor -- [1] thin
                               // forward to MotionEngine::
                               // setTurnFloor()/turnFloor().
    {"ramp_ms", 27},           // ConfigField.RampMs -- [ms] thin
    {"jerk", 28},          // ConfigField.Jerk
    {"plateau_min_s", 29}, // ConfigField.PlateauMinS
    {"max_yaw_rate", 30},  // ConfigField.MaxYawRate
    {"profile_exit", 31},  // ConfigField.ProfileExit
                               // forward to MotionEngine::
                               // setRampMs()/rampMs().
};
constexpr size_t kFieldCount = sizeof(kFields) / sizeof(kFields[0]);

const FieldEntry* findField(const char* name) {
  for (const auto& entry : kFields) {
    if (std::strcmp(name, entry.name) == 0) return &entry;
  }
  return nullptr;
}

// LOCAL flags layout, mirroring radio-robot-lib's own DiffDriveAdapter
// posture (diffdrive_adapter.cpp's computeFlags()): these bit numbers
// are NOT any externally-numbered scheme -- they exist only so STATUS's
// `flags=<hex>` packs the same handful of diagValue() booleans a bench
// operator already reads off the DIAG verb into one word.
constexpr uint32_t kFlagReady = 1u << 0;
constexpr uint32_t kFlagEstopped = 1u << 1;
constexpr uint32_t kFlagStallHalted = 1u << 2;
constexpr uint32_t kFlagLeaseExpired = 1u << 3;
constexpr uint32_t kFlagConnLeft = 1u << 4;
constexpr uint32_t kFlagConnRight = 1u << 5;
constexpr uint32_t kFlagWedgeLeft = 1u << 6;
constexpr uint32_t kFlagWedgeRight = 1u << 7;

// diagValue()'s own field-ordinal contract (shims.cpp) this file reads
// from -- named here so status() below reads as prose, not magic
// numbers. Only the subset status() actually needs; shims.cpp's
// probe() shim exposes many more diagValue() ordinals to the TS layer
// for on-device instrumentation (the cleartext DIAG verb this table
// used to back was retired).
constexpr int kDiagReady = 0;
constexpr int kDiagEstopped = 1;
constexpr int kDiagStallHalted = 2;
constexpr int kDiagLeaseExpired = 3;
constexpr int kDiagConnLeft = 4;
constexpr int kDiagConnRight = 5;
constexpr int kDiagWedgeLeft = 6;
constexpr int kDiagWedgeRight = 7;
constexpr int kDiagVelocityLeft = 14;
constexpr int kDiagVelocityRight = 15;

// Sprint 004 ticket 004 additions -- otosGet(7)'s connected/disconnected
// boolean (R-22/WIRE-06's fix for status()'s own out.otos, below) and
// diagValue()'s numeric FULL-column ordinals (sprint.md Phase B / the
// issue's own column table; shims.cpp's diagValue() switch is the
// authority for these numbers).
constexpr int kOtosConnected = 7;      // otosGet(int), NOT diagValue()
constexpr int kDiagI2cFault = 8;       // -> STATUS `i2cf=` and the `i2cf` column
constexpr int kDiagLeaseExpiryCount = 9;   // -> `lexc`
constexpr int kDiagPositionLeft = 10;      // -> `posl`
constexpr int kDiagPositionRight = 11;     // -> `posr`
constexpr int kDiagAppliedDutyLeft = 12;   // -> `dutl`
constexpr int kDiagAppliedDutyRight = 13;  // -> `dutr`
constexpr int kDiagCycleCount = 16;        // -> `cyc`
constexpr int kDiagCycleOverrunCount = 19; // -> `cycovr`
constexpr int kDiagWrongWayCount = 25;     // -> `wrng`

// LOCAL flags layout's own free function, extracted (unchanged bit
// layout) so STATUS's `flags=` and the telemetry `flags` column read
// the SAME computation instead of two independently-editable copies
// (sprint.md's own Design Rationale; status-lost-diag-numeric-surface.md).
// Called from both status() and buildSnapshot() below.
uint32_t computeFlags() {
  uint32_t flags = 0;
  if (diagValue(kDiagReady) != 0) flags |= kFlagReady;
  if (diagValue(kDiagEstopped) != 0) flags |= kFlagEstopped;
  if (diagValue(kDiagStallHalted) != 0) flags |= kFlagStallHalted;
  if (diagValue(kDiagLeaseExpired) != 0) flags |= kFlagLeaseExpired;
  if (diagValue(kDiagConnLeft) != 0) flags |= kFlagConnLeft;
  if (diagValue(kDiagConnRight) != 0) flags |= kFlagConnRight;
  if (diagValue(kDiagWedgeLeft) != 0) flags |= kFlagWedgeLeft;
  if (diagValue(kDiagWedgeRight) != 0) flags |= kFlagWedgeRight;
  return flags;
}

const char* tlmModeWireName(Wire::TlmMode mode) {
  switch (mode) {
    case Wire::TlmMode::kOff: return "off";
    case Wire::TlmMode::kPose: return "pose";
    case Wire::TlmMode::kFull: return "full";
    case Wire::TlmMode::kAuto: return "auto";
    // kBuffer can never reach mode_ (onTlm() below refuses it,
    // kUnimplemented, before assignment -- sprint 008 ticket 005), so
    // this arm is as unreachable via status()/buildSnapshot() as kNow's,
    // immediately below -- kept only so this switch stays exhaustive
    // against a future TlmMode enumerator.
    case Wire::TlmMode::kBuffer: return "buffer";
    // kNow is a one-shot request in the CURRENT mode's own shape
    // (protocol.md S6.1) -- never stored into mode_ (see onTlm() below),
    // kept here only so this switch stays exhaustive against a future
    // TlmMode enumerator.
    case Wire::TlmMode::kNow: return "pose";
  }
  return "off";  // unreachable with every enumerator handled above
}

// motion-api.md S9.1: "Angles are degrees at the API and milliradian
// integers on the wire ... The conversion lives in the binding, in one
// place." This is that one place: MOVE_X's wire `rotation` field and
// MOVE_V's wire `omega` field (sprint 003 ticket 012) both arrive here
// as a milliradian INTEGER, already decoded into this float by
// wire_handler.cpp; MotionEngine::moveX()/moveV() (motion_engine.h) both
// want RADIANS (or radians-per-second, for `omega`), their own native
// unit, with no wire-unit awareness of their own. 1 mrad == 0.001 rad,
// exact for any value either field can carry -- see
// test_wire_motion_verbs.py's own dedicated round-trip tests (both
// signs, both fields): an off-by-1000 here is invisible in a green
// build (everything still compiles and dispatches) and catastrophic on
// the robot (a 90 deg turn either barely twitches or spins wildly past
// a full revolution, depending on which way the factor is missed).
float mradToRad(float milliradians) { return milliradians * 0.001f; }

// SUC-003: the one place onMoveX()/onGoToR()/onGoToW() below resolve
// their own `cruise`/`speed == 0` "use the default" sentinel. Shaped
// mode (aDecelMmS2_ > 0, engineADecelMmS2()) resolves from THIS call's
// own leg distance via engineDefaultCruiseForDistanceMmS() -- which
// reads the SAME aDecelMmS2_/vMaxMmS_/brakeFrac_ the taper itself uses,
// so the two can never drift apart; legacy mode (aDecelMmS2_ == 0)
// keeps today's flat engineDefaultCruiseMmS(), unconditionally
// unchanged. onWheelsX()/onWheelsV() below do NOT call this helper --
// they keep the flat sentinel regardless of aDecelMmS2_ (wheelsX()'s
// two per-wheel distances have no single "leg length" this formula is
// defined over).
float resolveDefaultCruiseMmS(float distanceMm) {
  return engineADecelMmS2() > 0.0f
             ? engineDefaultCruiseForDistanceMmS(distanceMm)
             : engineDefaultCruiseMmS();
}

}  // namespace

WireAdapter::WireAdapter(const Wire::Identity& identity, NowMsFn nowMs)
    : identity_(identity), nowMs_(nowMs) {}

void WireAdapter::identity(Wire::Identity& out) const { out = identity_; }

void WireAdapter::setIdentity(const Wire::Identity& identity) {
  identity_ = identity;
}

uint32_t WireAdapter::now() const {
  // See this file's own header comment: nowMs_ is supplied at
  // composition time by a CODAL-facing caller (protocol.cpp); every
  // host test leaves it nullptr, so this stays the same honest 0
  // default it always was -- PING's own liveness contract only needs a
  // reply to exist, not a wall-clock-accurate value.
  return nowMs_ != nullptr ? nowMs_() : 0;
}

// Sprint 004 ticket 004 closes the numeric half of the gap the comment
// below used to describe (status-lost-diag-numeric-surface.md):
// `i2cf=<n>` now rides STATUS via the SAME diagValue(kDiagI2cFault) call
// the telemetry `i2cf` column reads, so the two can never disagree.
// FULL's other seven numeric columns (posl/posr/dutl/dutr/lexc/wrng/
// cycovr) still have no STATUS-level equivalent -- they are telemetry,
// not status, per the issue's own resolution of that split (see
// buildSnapshot() below); only `i2cf` was judged genuinely status-shaped
// (SUC-005). R-22/WIRE-06 (code review 2026-08-23) also fixed here:
// `out.otos` used to hardcode false with a comment claiming no OTOS was
// wire-reachable -- false even at the time it was written, since
// otosGet(7) already existed and engineGoToW() already gated on it.
void WireAdapter::status(Wire::StatusFields& out) const {
  out.ready = diagValue(kDiagReady) != 0;
  const bool estopped = diagValue(kDiagEstopped) != 0;
  const bool stallHalted = diagValue(kDiagStallHalted) != 0;
  const bool leaseExpired = diagValue(kDiagLeaseExpired) != 0;

  out.connLeft = diagValue(kDiagConnLeft) != 0;
  out.connRight = diagValue(kDiagConnRight) != 0;
  // R-22/WIRE-06 fix: otosGet(7) is the SAME connected/disconnected
  // boolean engineGoToW() (shims.cpp) already gates its own dispatch on
  // -- STATUS can no longer claim "no OTOS" while a GO_TO_W move is
  // actively using one. otosGet() reads a cache; this is not a
  // fresh-sample blocking read (see this file's own forward-declaration
  // block for why that distinction is load-bearing).
  out.otos = otosGet(kOtosConnected) != 0;
  out.wedge = diagValue(kDiagWedgeLeft) != 0 || diagValue(kDiagWedgeRight) != 0;
  // "active": ready, not halted, and a wheel is measurably moving.
  out.active = out.ready && !estopped && !leaseExpired && !stallHalted &&
               (diagValue(kDiagVelocityLeft) != 0 ||
                diagValue(kDiagVelocityRight) != 0);

  out.flags = computeFlags();
  out.i2cf = diagValue(kDiagI2cFault);
  // Sprint 010 ticket 003: same seam as i2cf immediately above, same
  // reasoning -- diagValue(kDiagCycleCount) is the SAME call FULL
  // telemetry's `cyc` column reads (buildSnapshot() below), so a
  // never-ticked kernel (cyc == 0) and a ticked-but-unreachable-brick
  // kernel (cyc > 0, connL/connR possibly still 0) are distinguishable
  // from STATUS alone (unpowered-nezha-brick-wedges-program-at-boot.md).
  out.cyc = static_cast<uint32_t>(diagValue(kDiagCycleCount));

  out.tlm = tlmModeWireName(mode_);
}

Wire::Result WireAdapter::onWheelsV(float left, float right,
                                    uint32_t duration, uint32_t id) {
  if (duration > kWheelsVDurationCeiling) return Wire::Result::kRange;
  // WIRE-08 (code review 2026-08-23): refuse BEFORE the cast below runs
  // at all -- see kWireBoundaryCastCeiling's own doc comment
  // (wire_adapter.h) for why an unclamped static_cast<int> here is
  // platform-dependent UB for a wire-legal but absurd value, and why
  // this bound is what keeps the cast well-defined (and therefore
  // identical) on both the C++20 host and the C++11 target.
  if (left < -kWireBoundaryCastCeiling || left > kWireBoundaryCastCeiling ||
      right < -kWireBoundaryCastCeiling || right > kWireBoundaryCastCeiling) {
    return Wire::Result::kRange;
  }
  // left/right arrive as exact integral values (decoded from the wire's
  // signed-integer fields, wire_handler.cpp) -- a plain narrowing cast
  // is exact, no rounding needed, now that the range check above rules
  // out the one region where "exact" stops being true.
  //
  // sprint 005 ticket 004: resolve any still-pending PREVIOUS motion as
  // "superseded" BEFORE dispatching this one, not after -- setWheelsTimed()
  // below calls MotionEngine::wheelsV(), whose FIRST act is cancelMove()
  // (motion-api.md S6: "wheels_* clears the planner"); if the previous
  // pending motion was goal-directed (MOVE_X/GO_TO_R/GO_TO_W),
  // resolvePendingReason() would misread that SIDE-EFFECT cancellation
  // as the old move having reached its own stop condition (kStop)
  // instead of having been superseded (kAborted) if this ran AFTER the
  // dispatch below. A no-op if nothing was pending (e.g. this is the
  // session's first motion verb, or no clock is wired).
  if (nowMs_ != nullptr) forceResolvePending(Wire::DoneReason::kAborted);
  setWheelsTimed(static_cast<int>(left), static_cast<int>(right), duration);
  // Record the resulting deadline so protocol.cpp's fiber loop keeps
  // ticking the kernel until it elapses; a no-op with no clock wired.
  if (nowMs_ != nullptr) {
    motionObligationActive_ = true;
    motionObligationDeadlineMs_ = nowMs_() + duration;
    armPendingMotion(id, /*goalDirected=*/false);
  }
  return Wire::Result::kOk;
}

Wire::Result WireAdapter::onWheelsX(float left, float right, float cruise,
                                    uint32_t timeout, uint32_t id) {
  // A speed ceiling has no sign -- refuse outright rather than take its
  // magnitude, or fall into wheelsX()'s own non-positive-cruise no-op
  // (motion_engine.h), which would silently accept this as "nothing to
  // command."
  if (cruise < 0.0f) return Wire::Result::kRange;
  // motion-api.md S1.1: "An X-form's commanded value is a displacement
  // ... so cruise is its own argument. Pass 0 for the configured
  // default." engineDefaultCruiseMmS() itself resolves to 0 if this
  // robot has never had one configured either -- refused below, not
  // silently accepted as a zero-speed command.
  const float resolvedCruise =
      cruise == 0.0f ? engineDefaultCruiseMmS() : cruise;
  if (resolvedCruise <= 0.0f) return Wire::Result::kRange;
  // sprint 005 ticket 004: see onWheelsV()'s identical comment above --
  // engineWheelsX() below routes onto MotionEngine::wheelsX(), which
  // ALSO calls cancelMove() first, same "wheels_* clears the planner"
  // rationale.
  if (nowMs_ != nullptr) forceResolvePending(Wire::DoneReason::kAborted);
  engineWheelsX(left, right, resolvedCruise, timeout);
  // timeout is a backstop, not the real duration -- conservative
  // deadline; ticking a little past completion is harmless.
  if (nowMs_ != nullptr) {
    motionObligationActive_ = true;
    motionObligationDeadlineMs_ = nowMs_() + timeout;
    armPendingMotion(id, /*goalDirected=*/false);
  }
  return Wire::Result::kOk;
}

Wire::Result WireAdapter::onMoveX(float distance, float rotation,
                                  float cruise, uint32_t timeout,
                                  uint32_t id) {
  // Same cruise <0 handling as onWheelsX() above; the ==0 substitution
  // itself now goes through resolveDefaultCruiseMmS() (SUC-003) instead
  // of the flat engineDefaultCruiseMmS() directly. D is
  // engineDominantAxisTravelMm(distance, rotationRad), NOT |distance|
  // alone -- a pure pivot (distance == 0) still moves its wheels
  // |rotationRad|*b/2 mm each, and resolving from |distance| alone
  // would always see D == 0 there and refuse every default-speed pivot
  // once shaped mode is on (every pivot in a tour is exactly this
  // call). `rotationRad` is computed once, here, and reused below for
  // the dispatch -- the wire's ONE milliradian->radian conversion seam
  // (motion-api.md S9.1, mradToRad()'s own comment above).
  if (cruise < 0.0f) return Wire::Result::kRange;
  const float rotationRad = mradToRad(rotation);
  const float resolvedCruise =
      cruise == 0.0f
          ? resolveDefaultCruiseMmS(
                engineDominantAxisTravelMm(distance, rotationRad))
          : cruise;
  if (resolvedCruise <= 0.0f) return Wire::Result::kRange;
  // sprint 005 ticket 004: resolve any still-pending PREVIOUS motion
  // BEFORE dispatching -- unlike WHEELS_V/WHEELS_X/MOVE_V,
  // engineMoveX() below does not call cancelMove() (it overwrites this
  // engine's own `move_` state directly, motion_engine.cpp), so this
  // ordering is not strictly required for correctness the way it is
  // there, but is kept identical for one uniform rule (resolve-before-
  // dispatch, always) rather than a rule with silent exceptions.
  if (nowMs_ != nullptr) forceResolvePending(Wire::DoneReason::kAborted);
  engineMoveX(distance, rotationRad, resolvedCruise, timeout);
  if (nowMs_ != nullptr) {
    // GOAL-DIRECTED: this class's own resolvePendingReason() also reads
    // engineMoveActive() for this one -- see wire_adapter.h.
    motionObligationActive_ = true;
    motionObligationDeadlineMs_ = nowMs_() + timeout;
    armPendingMotion(id, /*goalDirected=*/true);
  }
  return Wire::Result::kOk;
}

Wire::Result WireAdapter::onMoveV(float v_x, float omega, uint32_t duration,
                                  uint32_t id) {
  // Shares WHEELS_V's own ceiling and "duration is the lease" rationale
  // -- see kWheelsVDurationCeiling's own doc comment (wire_adapter.h).
  if (duration > kWheelsVDurationCeiling) return Wire::Result::kRange;
  // sprint 005 ticket 004: see onWheelsV()'s identical comment above --
  // engineMoveV() below routes onto MotionEngine::moveV(), which itself
  // calls wheelsV() (motion_engine.cpp), so the SAME cancelMove()
  // side-effect ordering hazard applies here.
  if (nowMs_ != nullptr) forceResolvePending(Wire::DoneReason::kAborted);
  // The wire's OTHER milliradian->radian conversion seam (motion-api.md
  // S9.1) -- `omega` is angle-shaped exactly like MOVE_X's `rotation`;
  // see mradToRad()'s own comment above.
  engineMoveV(v_x, mradToRad(omega), duration);
  // duration IS the lease already, same as onWheelsV().
  if (nowMs_ != nullptr) {
    motionObligationActive_ = true;
    motionObligationDeadlineMs_ = nowMs_() + duration;
    armPendingMotion(id, /*goalDirected=*/false);
  }
  return Wire::Result::kOk;
}

Wire::Result WireAdapter::onGoToR(float x, float y, float speed, float arrive,
                                  uint32_t timeout, uint32_t id) {
  // `speed`'s <0 handling mirrors onWheelsX()/onMoveX() above -- see
  // this method's own doc comment (wire_adapter.h) for why (`speed`
  // plays `cruise`'s role for the underlying moveX() call). The ==0
  // substitution goes through resolveDefaultCruiseMmS() (SUC-003),
  // using this call's own chord length (`x`, `y` are body-frame,
  // motion_engine.h's own goToR() doc comment) as D -- exactly the
  // distance goToR() itself will drive, whether it takes the plain-arc
  // or the pivot-then-chord split (motion_engine.cpp): the split drives
  // this SAME chord exactly, and the plain arc's own arc length is
  // never shorter than its chord, so resolving from the chord is never
  // optimistic about how far this move can actually brake.
  if (speed < 0.0f) return Wire::Result::kRange;
  const float resolvedSpeed =
      speed == 0.0f ? resolveDefaultCruiseMmS(std::hypot(x, y)) : speed;
  if (resolvedSpeed <= 0.0f) return Wire::Result::kRange;
  // sprint 005 ticket 004: see onMoveX()'s identical comment above --
  // GOAL-DIRECTED, same as MOVE_X.
  if (nowMs_ != nullptr) forceResolvePending(Wire::DoneReason::kAborted);
  engineGoToR(x, y, resolvedSpeed, arrive, timeout);
  if (nowMs_ != nullptr) {
    motionObligationActive_ = true;
    motionObligationDeadlineMs_ = nowMs_() + timeout;
    armPendingMotion(id, /*goalDirected=*/true);
  }
  return Wire::Result::kOk;
}

Wire::Result WireAdapter::onGoToW(float x, float y, float speed, float arrive,
                                  uint32_t timeout, uint32_t id) {
  // Same speed <0 handling as onGoToR() above, and the same
  // resolveDefaultCruiseMmS() substitution on ==0 (SUC-003) -- but
  // UNLIKE onGoToR()'s (x, y) (already body-frame, i.e. already the
  // chord to travel), this call's (x, y) are WORLD-frame absolute
  // target coordinates (goToW()'s own doc comment, motion_engine.h), so
  // D comes from engineGoToWChordMm(x, y) -- the TRUE body-frame chord
  // from the robot's CURRENT pose, using the exact same PoseSource
  // selection the dispatch below will use -- rather than hypot(x, y)
  // of the target alone, which is the target's distance from the world
  // ORIGIN and not this call's actual travel distance whenever the
  // robot is not sitting at the origin.
  if (speed < 0.0f) return Wire::Result::kRange;
  const float resolvedSpeed =
      speed == 0.0f ? resolveDefaultCruiseMmS(engineGoToWChordMm(x, y))
                    : speed;
  if (resolvedSpeed <= 0.0f) return Wire::Result::kRange;
  // Sprint 006 ticket 007: engineGoToW() now falls back to encoder
  // odometry when no OTOS is connected (motion-api.md S3.6) rather than
  // refusing, so this call always dispatches in the current
  // implementation -- see this method's own doc comment (wire_adapter.h)
  // for why the `!engineGoToW(...)` check below is nonetheless kept.
  if (!engineGoToW(x, y, resolvedSpeed, arrive, timeout)) {
    return Wire::Result::kUnimplemented;
  }
  // only armed on the path that actually dispatched a move.
  if (nowMs_ != nullptr) {
    // sprint 005 ticket 004: GOAL-DIRECTED, same as MOVE_X/GO_TO_R --
    // but UNLIKE those two (and unlike the three lease-style verbs),
    // this supersede resolution stays AFTER the dispatch call above,
    // not before: whether this GO_TO_W call dispatches at all is
    // exactly what `!engineGoToW(...)` above just decided, so there is
    // no "before dispatch" point at which that is already known. This
    // is still safe -- goToR()/goToW() (unlike wheelsV()/wheelsX())
    // never clears a PREVIOUS in-flight move's own `active` flag as a
    // side effect; it only ever overwrites `move_` with a real new
    // segment (which stays active) or leaves `move_` untouched entirely
    // (the `arrive`-gate no-op case, motion_engine.cpp) -- so a
    // still-live previous goal-directed pending motion's
    // engineMoveActive() reads correctly here regardless of ordering.
    forceResolvePending(Wire::DoneReason::kAborted);
    motionObligationActive_ = true;
    motionObligationDeadlineMs_ = nowMs_() + timeout;
    armPendingMotion(id, /*goalDirected=*/true);
  }
  return Wire::Result::kOk;
}

void WireAdapter::onEstop() {
  // ESTOP -> estopAll() -> kernel.estop() + emergencyStopMotors() +
  // engine.endMove(): the handler itself never inspects this method's
  // return (void, per wire_handler.h's own Adapter::onEstop() contract)
  // -- it replies `estop` unconditionally after calling this.
  estopAll();
  // sprint 005 ticket 004: resolve whatever motion was still pending as
  // kEstop, RIGHT NOW, UNCONDITIONALLY -- deliberately NOT going through
  // forceResolvePending()'s usual "trust resolvePendingReason()'s own
  // natural resolution first" path (every other force-resolve call site
  // in this file does), because a real ESTOP has two hazards that path
  // cannot see past:
  //   1. diagValue(kDiagEstopped) is a published Output field
  //      (shims.cpp's publishOutput(), mirrored by the host test
  //      double) that only updates on the kernel's NEXT step() -- it
  //      cannot yet read true here, no matter how real this estop is.
  //   2. estopAll() above already force-ends any in-flight goal-directed
  //      move (engine.endMove()), so engineMoveActive() already reads
  //      false for a pending MOVE_X/GO_TO_R/GO_TO_W -- resolvePendingReason()
  //      would misread that as "reached its own stop condition" (kStop)
  //      rather than "estopped," since hasLiveMotionObligation() is
  //      still true at this exact instant (see point 3 below).
  // ESTOP is this class's own highest-priority reason
  // (resolvePendingReason()'s own priority order checks it before
  // anything else, including kStall) -- there is never a MORE specific
  // NATURAL reason for this call to defer to the way onStop() below
  // correctly defers to an already-latched kStall, so committing
  // unconditionally is not a shortcut, it is the correct rule for this
  // one reason.
  if (pendingActive_) {
    lastDoneId_ = pendingId_;
    lastDoneReason_ = Wire::DoneReason::kEstop;
    pendingActive_ = false;
  }
  // 3. Cleared AFTER the commit above, not before: motionObligationActive_
  // feeds hasLiveMotionObligation(), which a NATURAL (non-ESTOP)
  // resolution above would have needed to read correctly -- clearing it
  // first would corrupt that read for no benefit, since this commit
  // does not consult it anyway. An e-stop must still revert
  // protocol.cpp's fiber loop to its idle poll immediately, not keep
  // ticking until a now-meaningless deadline elapses (same rationale
  // sprint 002's original obligation-clearing handleEstop() documented).
  motionObligationActive_ = false;
}

Wire::Result WireAdapter::onStop(bool /*immediate*/, uint32_t /*id*/) {
  // STOP [now] -> stopAll() -> kernel.neutral() + engine.endMove():
  // stopAll() has no refusal path of its own (matches kernel.neutral()'s
  // own unconditional acceptance), so this always acks kOk. `immediate`
  // (STOP's optional `now` token) has no effect here -- this project's
  // stopAll() has always been immediate regardless, same posture
  // DiffDriveAdapter documents for its own onStop() override
  // (protocol.md S5.1: "both are immediate at the kernel level").
  stopAll();
  // sprint 005 ticket 004: an explicit STOP ends whatever motion was
  // still pending -- "the stop condition was met, or stop() ended it"
  // (Wire::DoneReason::kStop's own doc comment, wire_handler.h).
  // forceResolvePending() gives resolvePendingReason() first refusal, so
  // an already-stalled pending motion keeps kStall instead of being
  // downgraded to kStop; for a goal-directed pending motion,
  // stopAll()'s own engine.endMove() above has already made
  // engineMoveActive() read false, so resolvePendingReason() naturally
  // resolves this to kStop on its own too (no forced fallback actually
  // needed there, but it is harmless -- unlike ESTOP above, there is no
  // hazard here: nothing this call does can be mistaken for a DIFFERENT
  // reason, since kStop is exactly what a false engineMoveActive() with
  // a still-live deadline already means). Called BEFORE clearing
  // motionObligationActive_ below -- clearing it first would make
  // hasLiveMotionObligation() read false prematurely and misresolve a
  // still-pending LEASE-STYLE motion (WHEELS_V/WHEELS_X/MOVE_V, which
  // has no engineMoveActive() signal of its own to fall back on) as
  // kTimeout instead of kStop.
  forceResolvePending(Wire::DoneReason::kStop);
  // see onEstop()'s identical comment above.
  motionObligationActive_ = false;
  return Wire::Result::kOk;
}

bool WireAdapter::hasLiveMotionObligation() const {
  if (!motionObligationActive_ || nowMs_ == nullptr) return false;
  const uint32_t nowMs = nowMs_();
  // Wraparound-safe elapsed check (signed-difference idiom), same one
  // sprint 002's original obligation tracking used in protocol.cpp.
  return static_cast<int32_t>(nowMs - motionObligationDeadlineMs_) < 0;
}

// ---- sprint 005 ticket 004: motion-completion resolution (S8.8) --------
// See wire_adapter.h's own comment on lastDone()/lastDoneReason() for the
// full design; this is the implementation.

Wire::DoneReason WireAdapter::resolvePendingReason() const {
  if (!pendingActive_) return Wire::DoneReason::kNone;
  // Safety conditions win regardless of verb kind or deadline state --
  // wherever a panic stop or a kernel-level stall lands, it is the
  // right answer for whatever motion was in flight when it happened.
  // Both already reach this class through the SAME diagValue() path
  // computeFlags() uses for STATUS's `flags=` and telemetry's `flags`
  // column -- no new plumbing (sprint.md's own Design Rationale).
  if (diagValue(kDiagEstopped) != 0) return Wire::DoneReason::kEstop;
  if (diagValue(kDiagStallHalted) != 0) return Wire::DoneReason::kStall;
  if (pendingGoalDirected_) {
    // MOVE_X/GO_TO_R/GO_TO_W: the ONE genuinely new read (this file's
    // own forward declaration above). Still active means not yet
    // resolved, regardless of the wire-side deadline -- the engine's
    // own internal deadline/goal/wrong-way checks (serviceMove(),
    // motion_engine.cpp) are what eventually clear it, and this class
    // only ever observes the result. Once it goes inactive: the wire-
    // side lease not yet elapsed means it reached its own stop
    // condition EARLY (kStop); already elapsed means the deadline is
    // what ended it (kTimeout).
    if (engineMoveActive()) return Wire::DoneReason::kNone;
    return hasLiveMotionObligation() ? Wire::DoneReason::kStop
                                      : Wire::DoneReason::kTimeout;
  }
  // WHEELS_V/WHEELS_X/MOVE_V: no engine read needed -- these resolve
  // entirely from the SAME lease-deadline bookkeeping
  // hasLiveMotionObligation() already provides ("no new dependency
  // needed for these three verbs," sprint.md's own Design Rationale).
  // A still-live lease means not yet resolved; an elapsed one with
  // nothing having superseded or stopped it (both handled elsewhere,
  // see forceResolvePending()) means it simply ran out the clock.
  return hasLiveMotionObligation() ? Wire::DoneReason::kNone
                                    : Wire::DoneReason::kTimeout;
}

void WireAdapter::resolvePendingIfDue() const {
  if (!pendingActive_) return;
  const Wire::DoneReason reason = resolvePendingReason();
  if (reason == Wire::DoneReason::kNone) return;  // not yet terminal
  lastDoneId_ = pendingId_;
  lastDoneReason_ = reason;
  pendingActive_ = false;
  // sprint 016 ticket 003 (closing wire-motion-obligation-never-clears.md):
  // this commit already knows the motion is over -- the NATURAL-completion
  // clearing point that was missing before (only onEstop()/onStop() ever
  // cleared this flag). `reason` was computed by resolvePendingReason()
  // just above, which itself reads the pre-clear value of
  // motionObligationActive_ (via hasLiveMotionObligation()) for the
  // lease-style branch -- so clearing it AFTER that read, not before, is
  // required for correctness here too, same ordering rule onStop()'s own
  // comment documents for forceResolvePending() below.
  motionObligationActive_ = false;
}

void WireAdapter::forceResolvePending(Wire::DoneReason forcedReason) {
  if (!pendingActive_) return;
  // resolvePendingReason() gets first refusal: an already-stalled or
  // already-estopped pending motion keeps THAT more specific reason
  // rather than being overwritten by whatever the caller forces (e.g.
  // onStop()'s own kStop) -- see this method's own doc comment
  // (wire_adapter.h).
  Wire::DoneReason reason = resolvePendingReason();
  if (reason == Wire::DoneReason::kNone) reason = forcedReason;
  lastDoneId_ = pendingId_;
  lastDoneReason_ = reason;
  pendingActive_ = false;
  // sprint 016 ticket 003: same clearing this method's own doc comment
  // (wire_adapter.h) describes -- covers the "a later verb supersedes a
  // still-live earlier one" (kAborted) path, which runs BEFORE the *new*
  // verb re-arms motionObligationActive_ = true a few lines later in the
  // same onXxx() handler (wire_adapter.cpp's six onWheelsV()/onWheelsX()/
  // onMoveX()/onMoveV()/onGoToR()/onGoToW() bodies), so ordering stays
  // correct. onStop() also reaches this method (with kStop) and then sets
  // its own explicit motionObligationActive_ = false right after --
  // redundant with this line for that one caller, but harmless
  // (idempotent); onStop()'s own comment explains why that ordering
  // (forceResolvePending() BEFORE its explicit clear) still matters
  // independent of this method's own internal clear.
  motionObligationActive_ = false;
}

void WireAdapter::armPendingMotion(uint32_t id, bool goalDirected) {
  pendingActive_ = true;
  pendingId_ = id;
  pendingGoalDirected_ = goalDirected;
}

uint32_t WireAdapter::lastDone() const {
  resolvePendingIfDue();
  return lastDoneId_;
}

Wire::DoneReason WireAdapter::lastDoneReason() const {
  resolvePendingIfDue();
  return lastDoneReason_;
}

bool WireAdapter::onGet(const char* name, float& out) const {
  const FieldEntry* entry = findField(name);
  if (entry == nullptr) return false;
  // config values cross the shim boundary as x1000-scaled ints
  // (shims.cpp convention).
  out = static_cast<float>(getConfigValue(entry->ordinal)) * 0.001f;
  return true;
}

Wire::Result WireAdapter::onSet(const char* name, float value, uint32_t id) {
  (void)id;
  const FieldEntry* entry = findField(name);
  if (entry == nullptr) return Wire::Result::kUnknown;
  // WIRE-08 (code review 2026-08-23): `value` arrives with no ceiling
  // of its own (parseFloatField, wire_handler.cpp, accepts any finite
  // float) -- setKernelValue()'s own x1000 scaling convention can turn
  // an absurd-but-legal field value into a product that overflows
  // `long`'s 32-bit range before std::lround() ever runs (e.g. `SET
  // pid_kp 3000000` -> 3e9), which is unspecified on the target, not
  // merely imprecise. Refuse BEFORE the round -- see
  // kWireBoundaryCastCeiling's own doc comment (wire_adapter.h).
  const float scaled = value * 1000.0f;
  if (scaled < -kWireBoundaryCastCeiling || scaled > kWireBoundaryCastCeiling) {
    return Wire::Result::kRange;
  }
  setKernelValue(entry->ordinal, static_cast<int>(std::lround(scaled)));
  return Wire::Result::kOk;
}

size_t WireAdapter::fieldCount() const { return kFieldCount; }

const char* WireAdapter::fieldName(size_t index) const {
  return index < kFieldCount ? kFields[index].name : "";
}

Wire::Result WireAdapter::onTlm(Wire::TlmMode mode) {
  // Sprint 008 ticket 005 (closing tlm-auto-buffer-column-set-undefined.md):
  // TLM BUFFER refuses -- kUnimplemented, wire err 6 -- right here, BEFORE
  // mode_ is ever touched: no buffering mechanism exists anywhere in this
  // codebase to give "buffer" real, narrower column-set semantics yet
  // (DESIGN.md S14's own Design Rationale), so answering err is more
  // honest than emitting a column set nobody specified. This is a MERITS
  // rejection, not a decode failure (decodeTlm()/parseTlmMode() already
  // accept "BUFFER" as a well-formed token) -- same "ack unconditionally,
  // then err on top, state left unchanged" shape wire_handler.cpp's
  // clampMotionTimeout()-based refusals already established for the six
  // motion verbs (sprint 008 ticket 001). mode_ is left exactly as it
  // was, so a host with e.g. TLM POSE already active keeps getting POSE
  // frames after a refused TLM BUFFER -- it is NOT silently switched to
  // off or to any other mode.
  if (mode == Wire::TlmMode::kBuffer) return Wire::Result::kUnimplemented;
  // TLM NOW is a one-shot request in the CURRENT subscription's shape,
  // not a new subscription (protocol.md S6.1: "does not change mode") --
  // so it is deliberately never stored into mode_. TLM AUTO is a
  // documented ALIAS for TLM POSE as of this same ticket -- it stores
  // mode_ = kAuto here like any other mode, but buildSnapshot() below
  // already treats every stored mode other than kFull identically, so
  // kAuto needs no separate branch there to behave exactly like kPose
  // (same 12 columns, same cadence). Everything else (kOff/kPose/kFull/
  // kAuto) becomes the persisted mode.
  if (mode != Wire::TlmMode::kNow) mode_ = mode;
  return Wire::Result::kOk;
}

bool WireAdapter::telemetryEnabled() const {
  return mode_ != Wire::TlmMode::kOff;
}

const Wire::Snapshot& WireAdapter::buildSnapshot() {
  // protocol.md S6.2: seq wraps 0..127 -- THIS method's responsibility,
  // not WireHandler's (the handler only prints whatever value it is
  // given). Advanced BEFORE building the frame, so the very first frame
  // this adapter ever emits reports seq 1, not 0.
  seq_ = static_cast<uint8_t>((seq_ + 1) & 0x7F);

  // Read every source exactly ONCE per call -- poseX()/poseY()/
  // poseHeading() each mutate odometry (this file's own
  // forward-declaration block), so calling any of them twice per tick
  // would double-advance it for no reason.
  const int32_t x = static_cast<int32_t>(poseX());
  const int32_t y = static_cast<int32_t>(poseY());
  const int32_t h = static_cast<int32_t>(poseHeading());
  // otosGet(0)/(1) are 0.1 mm -- divide by 10 for ox/oy (plain integer
  // division, truncating toward zero: the issue's own "OTOS 0.1mm->mm"
  // scale test pins -5678 -> -567, not the round-half -568 a
  // std::lround-style conversion would give). otosGet(2) is ALREADY
  // centidegrees -- do NOT also divide it.
  const int32_t ox = otosGet(0) / 10;
  const int32_t oy = otosGet(1) / 10;
  const int32_t oh = static_cast<int32_t>(otosGet(2));
  const int32_t vl = static_cast<int32_t>(wheelSpeed(0));
  const int32_t vr = static_cast<int32_t>(wheelSpeed(1));
  const int32_t i2cf = static_cast<int32_t>(diagValue(kDiagI2cFault));
  const uint32_t flags = computeFlags();

  size_t i = 0;
  columns_[i++] = {"seq", static_cast<int32_t>(seq_), false};
  columns_[i++] = {"now", static_cast<int32_t>(now()), false};
  columns_[i++] = {"flags", static_cast<int32_t>(flags), true};
  columns_[i++] = {"x", x, false};
  columns_[i++] = {"y", y, false};
  columns_[i++] = {"h", h, false};
  columns_[i++] = {"ox", ox, false};
  columns_[i++] = {"oy", oy, false};
  columns_[i++] = {"oh", oh, false};
  columns_[i++] = {"vl", vl, false};
  columns_[i++] = {"vr", vr, false};
  columns_[i++] = {"i2cf", i2cf, false};

  // FULL adds these 8; every other mode this adapter can actually reach
  // here is kPose or kAuto -- kAuto is a documented ALIAS for kPose
  // (sprint 008 ticket 005: same 12 columns, same cadence, no separate
  // branch needed); kOff never reaches this call at all (telemetryEnabled()
  // gates it, see protocol.cpp's fiber loop); kBuffer can never reach
  // mode_ in the first place -- onTlm() above refuses it before
  // assignment. See this file's own buildSnapshot() doc comment,
  // wire_adapter.h, for the full decision.
  if (mode_ == Wire::TlmMode::kFull) {
    columns_[i++] = {"cyc", static_cast<int32_t>(diagValue(kDiagCycleCount)),
                     false};
    columns_[i++] = {"posl",
                     static_cast<int32_t>(diagValue(kDiagPositionLeft)),
                     false};
    columns_[i++] = {"posr",
                     static_cast<int32_t>(diagValue(kDiagPositionRight)),
                     false};
    columns_[i++] = {"dutl",
                     static_cast<int32_t>(diagValue(kDiagAppliedDutyLeft)),
                     false};
    columns_[i++] = {"dutr",
                     static_cast<int32_t>(diagValue(kDiagAppliedDutyRight)),
                     false};
    columns_[i++] = {"lexc",
                     static_cast<int32_t>(diagValue(kDiagLeaseExpiryCount)),
                     false};
    columns_[i++] = {"wrng",
                     static_cast<int32_t>(diagValue(kDiagWrongWayCount)),
                     false};
    columns_[i++] = {"cycovr",
                     static_cast<int32_t>(diagValue(kDiagCycleOverrunCount)),
                     false};
  }

  snapshot_.columns = columns_;
  snapshot_.count = i;
  return snapshot_;
}

Wire::Result WireAdapter::onRun(const char* /*name*/,
                                const char* const* /*argv*/,
                                size_t /*argc*/, char* /*result*/,
                                size_t /*resultCapacity*/, bool& hasResult) {
  // No registration table -- see wire_adapter.h's own doc comment on
  // this override. Every RUN is ERR_UNKNOWN, the same wire outcome as
  // any name a real registration table would not recognize.
  hasResult = false;
  return Wire::Result::kUnknown;
}

}  // namespace diffDrive
