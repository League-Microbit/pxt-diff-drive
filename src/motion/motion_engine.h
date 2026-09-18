// diffDrive::MotionEngine -- two primitives and the reductions onto them.
// wheelsX() commands per-wheel DISTANCE, ratio-locked so both wheels finish
// together; wheelsV() commands per-wheel VELOCITY for a duration that IS
// the kernel's lease. moveX()/moveV()/goToR()/goToW() reduce onto those
// two, each clearing any in-flight command first.
//
// SIGN CONVENTION: CCW-positive. A positive twist turns LEFT and increases
// camera yaw; the left wheel is the slower one in a left turn. Never
// re-derive this from cable order -- a host test pins it.
//
// Grammar spec: radio-robot-lib/docs/design/motion-api.md (read-only, a
// different repo). Design and rationale: DESIGN.md.
#pragma once

#include <cstdint>

#include "../core/diffdrive.h"
#include "motion_limits.h"
#include "segment.h"
#include "velocity_shaper.h"

namespace diffDrive {

// A minimal world-pose read port for goToW(). Implemented by OtosPort
// (hardware), Odometry (encoder fallback) and FakePoseSource (tests).
class PoseSource {
 public:
  virtual ~PoseSource() = default;

  virtual float x() const = 0;  // [mm] world frame
  virtual float y() const = 0;  // [mm] world frame

  // [rad] world frame, CCW+. Wrap convention is IMPLEMENTATION-DEFINED:
  // OtosPort wraps to (-pi, pi], Odometry does not. Valid because goToR()/
  // goToW() consume this only through cos()/sin(). A caller that
  // DIFFERENCES two reads must not assume a shared convention.
  virtual float heading() const = 0;
};

class MotionEngine {
 public:
  // `kernel`/`clock` are owned by the caller; this class holds references
  // only. The Clock is separate from the kernel's own because shaping and
  // the timeout backstop need wall time whether or not the kernel stepped.
  MotionEngine(DiffDrive::DifferentialDrive& kernel,
               const DiffDrive::Clock& clock);

  // ---- geometry ----

  // [mm/deg] wheel travel per shaft degree.
  float travelCalib() const { return travelCalib_; }
  void setTravelCalib(float mmPerDeg) { travelCalib_ = mmPerDeg; }

  // [mm] the CALIPER-MEASURED track width. Never adjust it to correct a
  // turn -- all rotational correction belongs in rotationalSlip.
  float trackWidth() const { return trackWidth_; }
  void setTrackWidth(float mm) { trackWidth_ = mm; }

  // [1] physical/odometric rotation ratio (wheel-contact scrub),
  // camera-measured. Read DESIGN.md's derivation before setting a new
  // value: the obvious shortcut produces a plausible wrong number.
  float rotationalSlip() const { return rotationalSlip_; }
  void setRotationalSlip(float slip) {
    if (slip > 0.0f) rotationalSlip_ = slip;
  }

  // [counts/mm] 1 count == 0.1 shaft degree.
  float countsPerMm() const { return 10.0f / travelCalib_; }

  // [mm] b = trackWidth / rotationalSlip. A method, never a cached field,
  // so a config read-back cannot report a derived number as a measured one.
  float effectiveTrackWidth() const { return trackWidth_ / rotationalSlip_; }

  // [mm] -> [mm/s] the default cruise for a leg of this length, for the
  // `cruise == 0` wire sentinel: min(vMax, sqrt(decel * D)). Derived fresh
  // every call, same as effectiveTrackWidth().
  float defaultCruiseForDistance(float distance) const;

  // [mm] [rad] -> [mm] the dominant-axis wheel travel for (distance,
  // rotation), as input to defaultCruiseForDistance(). A pure pivot's
  // wheels genuinely travel |rotation| * b / 2 even though the chassis
  // does not translate.
  float dominantAxisTravel(float distance, float rotation) const;

  struct DualRateReconciliation {
    float cruise;        // [mm/s]
    float distDuration;  // [s] the distance axis at its own ceiling
    float yawDuration;   // [s] the yaw axis at its own ceiling
  };

  // The block API exposes two independent rate ceilings; every native entry
  // point takes one `cruise`. This collapses them: whichever axis takes
  // longer governs a shared duration, and `cruise` is the dominant wheel
  // speed reproducing that motion. Both axis durations are returned so a
  // caller whose move splits into phases can budget off their sum. `speed`
  // and `yawRate` must already be floored positive by the caller. All-zero
  // return means there is nothing to do.
  DualRateReconciliation reconcileDualRateCruise(
      float distance, float rotation, float speed,
      float yawRate) const;  // [mm] [rad] [mm/s] [rad/s]

  // ---- the two primitives ----

  // Hold each wheel at a velocity for `duration`, which IS the kernel's
  // lease. Clears the planner. Drives nothing synchronously: this arms the
  // hold, and the first command lands one service() tick later.
  void wheelsV(float left, float right, uint32_t duration);  // [mm/s] [mm/s] [ms]

  // Move each wheel a distance, ratio-locked to `cruise` (the DOMINANT
  // wheel's ceiling) so both finish together. Closed-loop on encoders;
  // `timeout` is a deadline backstop, never the stop condition. A
  // zero-magnitude command or non-positive cruise commands nothing new but
  // still stops motion already in progress. Clears the planner.
  void wheelsX(float left, float right, float cruise, uint32_t timeout);  // [mm] [mm] [mm/s] [ms]

  // [rad] the arc angle |theta| at which goToR() reaches its target by
  // pivot-then-chord instead of the tangent arc. moveX() has NO such
  // threshold: every (distance, rotation) is one blended arc. Exposed so a
  // caller mirroring goToR()'s decision reads it rather than re-typing it.
  static constexpr float turnFirstAngle() { return kTurnFirstAngle; }

  // goToR()'s bearing-then-chord decomposition, pure so a caller
  // reconciling a separate yaw-rate ceiling makes the identical split.
  struct GoToRPlan {
    float bearingRaw;  // [rad] atan2(y, x) -- the pivot angle when willSplit
    float theta;       // [rad] wrapped 2*bearingRaw -- the blended rotation
    float chord;       // [mm] hypot(x, y) -- the straight phase when willSplit
    float arcLength;   // [mm] the blended segment's signed distance
    bool willSplit;    // |theta| >= kTurnFirstAngle
  };
  static GoToRPlan decomposeGoToR(float x, float y);  // [mm] [mm]

  // ---- move engine ----

  // Supersedes any in-flight command. Always ONE blended constant-radius
  // segment, R = distance / rotation, for any pair -- including |R| < b/2
  // (inner wheel reversed) and |rotation| beyond a full turn. Never splits.
  void moveX(float distance, float rotation, float cruise,
             uint32_t timeout);  // [mm] [rad] [mm/s] [ms]

  // The plain wheelsV reduction, vx +- omega*b/2, held for `duration`.
  void moveV(float vx, float omega, uint32_t duration);  // [mm/s] [rad/s] [ms]

  // Drive to a body-frame target (`x` forward, `y` left). A target within
  // `arrive` of the current position is a no-op. Single-shot: a caller
  // wanting repeat-until-arrival re-issues the call. Makes its OWN
  // pivot-vs-blend split rather than inheriting moveX()'s -- see DESIGN.md.
  void goToR(float x, float y, float speed, float arrive,
             uint32_t timeout);  // [mm] [mm] [mm/s] [mm] [ms]

  // World-frame goToR(): reads `pose` ONCE, at call time, rotates the delta
  // into the body frame and delegates.
  void goToW(const PoseSource& pose, float x, float y, float speed,
             float arrive, uint32_t timeout);  // [mm] [mm] [mm/s] [mm] [ms]

  // The single per-tick advance: dispatches whichever of seg_/hold_ is
  // active through shaper_, at most one kernel drive()/neutral() per call.
  // Owns nothing about odometry -- callers update that around this call.
  bool service();

  // A position-mode Segment is in flight. A continuous hold does not count.
  bool isMoveActive() const { return seg_.active; }

  // Either a Segment or a Hold is driving. Callers inferring "something is
  // driving" must read this, not the kernel's applied duty, which is one
  // tick stale after a command is armed.
  bool isDriving() const { return seg_.active || hold_.active; }

  // The most recent Segment ended on its own deadline rather than by
  // arriving, aborting or being cancelled. Latched on the exact tick the
  // segment ended, so a later read still answers as of that tick; valid
  // until the next segment starts.
  bool lastSegmentEndedByDeadline() const {
    return lastSegmentEndedByDeadline_;
  }

  // Force-end the current command now; no-op if nothing is active.
  void endMove();

  // [0..1000] dominant-axis fraction completed; 1000 when no Segment is
  // active. A continuous hold has no notion of done and is unaffected.
  int progress() const;

  uint32_t wrongWayCount() const { return wrongWayCount_; }

  // A stall halted a command and no later command has shown the wheels
  // turning yet. The kernel's own stallHalted is the HALT, and every new
  // command re-arms it -- a stall stops only the command it happened in.
  // This is the REPORT: it stays set after the halt so a program can ask
  // afterwards, and clears on the first tick a later command measures
  // either wheel above the stall speed, or on clearStallReport().
  bool stallReported() const { return stallReported_; }
  void clearStallReport() { stallReported_ = false; }

  // Steps the kernel until both wheels MEASURE at rest, bounded. Needed
  // because neutral() only stages a zero command and the delivering step's
  // own encoder read can otherwise freeze a nonzero velocity forever.
  // Issues no command of its own and folds nothing into odometry.
  void settleToRest();

  // Per-wheel encoder-count delta a pulseWheels() call measured, across
  // the WHOLE call (pulse + hard-zero + settle) -- never a per-tick
  // sample, since this primitive drives its own tick loop synchronously
  // and returns only once, at the end.
  struct PulseResult {
    float left;   // [counts]
    float right;  // [counts]
  };

  // Fires ONE bounded-width, bounded-amplitude raw-duty pulse per wheel
  // via the kernel's own driveDuty() (kModeRawDuty -- already bypasses
  // PID, the speed floor, the crawl dither and twist-hold; E-stop and
  // lease expiry still force neutral through the kernel's own
  // controlStep(), unaffected by anything below), hard-zeros, then
  // reports each wheel's encoder-count delta once both wheels read at
  // rest (settleToRest()). This is pure diagnostic substrate for a
  // floor characterization gate -- no automatic looping, no re-read/
  // terminate logic. A settle-gated STEPPER that fires this repeatedly,
  // re-reading remaining error between pulses, is a deliberately
  // separate later concern -- not built here.
  //
  // `widthTicks` counts kernel.step() calls THIS METHOD DRIVES ITSELF,
  // synchronously, in a loop -- there is no tickDrive()/tick-engine
  // cadence involved anywhere in this call, unlike every other
  // MotionEngine primitive, which only ARMS a command for the tick
  // engine's own next step(). A caller reaching this on real hardware
  // must not have another fiber concurrently calling tickDrive() on the
  // same kernel -- shims.cpp's forward wraps this call in the same
  // BusGuard tickDrive() itself acquires, for exactly that reason.
  // `widthTicks <= 0` fires no pulse at all (still hard-zeros and
  // settles, reporting ~0 delta): a defensive no-op, not a refusal.
  //
  // `amp{Left,Right}` are duty PERCENT [-100, 100], the same scale
  // driveDuty() itself takes -- passed straight through, unclamped
  // here (the kernel's own controlStep() clamps to the configured
  // maxDuty rail). A single-tick pulse cannot exceed ~25% duty and a
  // two-tick pulse ~50% AT THE MOTOR PORT regardless of the amplitude
  // requested here, per the port's own 25%-per-tick slew limit
  // (src/platform/nezha_port.cpp) -- a hardware constraint on the
  // CALLER's choice of amplitude/width, not something this primitive
  // works around or compensates for.
  //
  // Clears any in-flight move-engine command first, same as every
  // other primitive here (wheelsV()/wheelsX()'s own "wheels_* clears
  // the planner").
  PulseResult pulseWheels(float ampLeft, float ampRight,
                          int32_t widthTicks);  // [%] [%] [ticks]

  // ---- nudge mode: the settle-gated pulse stepper ----------------------
  //
  // Loops pulseWheels()'s own underlying primitive (firePulseAndSettle()
  // below, factored out of pulseWheels() so both share one call
  // sequence): fires a pulse only on a tick where BOTH wheels already
  // read at rest (atRest(), the same test settleToRest() uses -- one
  // definition of "stopped", never a second), re-reads the remaining
  // error from encoder counts (never carried/accumulated state), and
  // terminates on margin, deadline, or a conservative pulse budget
  // (nudgeMaxPulses()) -- raw-duty mode updates no stall latch, so this
  // budget is the only runaway backstop (this method's own header
  // comment on pulseWheels() above).
  //
  // `distance`/`rotation` convert to (distTarget, yawTarget) COUNTS
  // exactly as moveX()/beginSegment() do: distTarget = distance *
  // countsPerMm(), yawTarget = rotation * 0.5 * effectiveTrackWidth() *
  // countsPerMm(). A pure straight nudge passes rotation == 0; a pure
  // turn (nudgeTurn()) passes distance == 0; nudge(leftMm, rightMm)
  // reduces its own per-wheel pair onto this same (distance, rotation)
  // pair the way wheelsX() already reduces onto beginSegment(). Clears
  // any in-flight seg_/hold_ command first, same
  // as every other primitive here -- exactly one of seg_/hold_/nudge_ is
  // ever live (isNudgeActive()).
  //
  // Amplitude (duty %), width (ticks) and the inter-pulse settle pause
  // (ms) are read from this engine's own nudgeAmplitude()/
  // nudgeWidthTicks()/nudgeSettle() at FIRE time, not captured here --
  // a wire `SET` mid-nudge takes effect on the very next pulse, the same
  // "config read fresh every use" contract every other MotionEngine
  // config knob (limits(), rotationalSlip()) already has.
  void beginNudge(float distance, float rotation,
                  uint32_t timeout);  // [mm] [rad] [ms]

  // A nudge target is in flight (service() is dispatching to
  // serviceNudge()). Mutually exclusive with isMoveActive()/hold_'s own
  // active flag -- cancelMove() clears all three.
  bool isNudgeActive() const { return nudge_.active; }

  // [pulses] how many pulses the MOST RECENT (or still in-flight) nudge
  // has fired -- serviceNudge() only clears nudge_.active on
  // termination, not the rest of the struct, so this stays readable
  // (and stops changing) immediately after a nudge ends. A fresh
  // beginNudge() resets it to 0.
  int32_t nudgePulseCount() const { return nudge_.pulsesFired; }

  // [pulses] the fixed, conservative pulse-budget backstop every nudge
  // is bound by (Design Rationale: "raw-duty pulses ride the kernel's
  // existing kModeRawDuty ... nudge stepper's own deadline + pulse
  // budget is the only runaway backstop"). Exposed so a test (or a
  // caller sizing its own deadline) reads the real number rather than
  // re-typing it.
  static constexpr int32_t nudgeMaxPulses() { return kMaxNudgePulses; }

  // ---- nudge mode: the measured result ----------------------------------
  //
  // nudge()/nudgeTurn() (blocks/motion.ts) must return what the encoders
  // ACTUALLY measured, not the requested amount -- the entire reason
  // calibrateL can loop on the return value instead of trusting the
  // command. These two accessors read straight from the CURRENT kernel
  // Output against the nudge's own captured origin (Nudge::posLeft0/
  // posRight0), the exact same ledger serviceNudge()'s own convergence
  // test (distRemain/yawRemain above) already uses -- never a
  // re-derivation through the
  // fused odometry pose, which lags a tick behind inside tickDrive()
  // (odomUpdate() runs BEFORE service() there). Valid any time after
  // beginNudge(), including after the nudge has ended: nudge_ is not
  // reset on termination (nudgePulseCount()'s own comment above), only
  // ever overwritten by the NEXT beginNudge().
  float nudgeMeasuredDistance() const {  // [mm] mean-axis, matches
                                          // beginNudge()'s own `distance`
    const DiffDrive::DifferentialDrive::Output out = kernel_.output();
    return 0.5f * ((out.positionLeft - nudge_.posLeft0) +
                   (out.positionRight - nudge_.posRight0)) /
           countsPerMm();
  }

  float nudgeMeasuredRotation() const {  // [deg] CCW+, inverse of
                                          // beginNudge()'s own yawTarget
                                          // formula
    const DiffDrive::DifferentialDrive::Output out = kernel_.output();
    const float yawDelta =  // [counts]
        0.5f * ((out.positionRight - nudge_.posRight0) -
                (out.positionLeft - nudge_.posLeft0));
    // yawTarget (counts) = rotation * 0.5 * effectiveTrackWidth() *
    // countsPerMm() -- beginNudge()'s own formula above -- so inverting
    // for rotation needs the factor of 2 back:
    // rotation = 2 * yawDelta / (effectiveTrackWidth() * countsPerMm()).
    const float rad =
        2.0f * yawDelta / (countsPerMm() * effectiveTrackWidth());
    return rad * (180.0f / 3.14159265f);
  }

  // [%] duty magnitude a nudge pulse fires at, applied to whichever
  // wheel(s) the remaining error's sign selects. Default 15.0f: MEASURED
  // vevov 2026-09-16, captures/039-003-pulse-gate-20260916/notes.md --
  // the accepted operating point (amplitude 15%, width 2 ticks,
  // 1.79 mm/pulse, sd/mean 0.08, 0/20 dead warm and cold).
  float nudgeAmplitude() const { return nudgeAmplitude_; }
  void setNudgeAmplitude(float percent) {
    if (percent > 0.0f) nudgeAmplitude_ = percent;
  }

  // [ticks] pulse width fired per nudge step. Default 2: same MEASURED
  // citation as nudgeAmplitude() above -- width 1 is the "nothing-or-
  // lurch" bimodal signature the gate rejected at every amplitude
  // tried; width 2 is what buys repeatability.
  int32_t nudgeWidthTicks() const { return nudgeWidthTicks_; }
  void setNudgeWidthTicks(int32_t ticks) {
    if (ticks > 0) nudgeWidthTicks_ = ticks;
  }

  // [ms] a wall-clock pause serviceNudge() honors, on top of the
  // encoder-velocity rest test (atRest()), before it will fire the next
  // pulse -- an operator-tunable margin for mechanical settling
  // (backlash, structural ring-down) the encoder alone may not show.
  // Default 0.0f (off): UNVERIFIED -- no hardware measurement backs a
  // nonzero default, so this knob does not change behavior until an
  // operator or a later characterization run sets one.
  float nudgeSettle() const { return nudgeSettle_; }
  void setNudgeSettle(float ms) {
    if (ms >= 0.0f) nudgeSettle_ = ms;
  }

  // The one settable shaping surface. This engine holds no shaping knob.
  MotionLimits& limits() { return limits_; }
  const MotionLimits& limits() const { return limits_; }

 private:
  // [rad] 50 deg. goToR() only: at or above this arc angle, pivot to the
  // bearing first, then drive the chord. Inherited from a go-to-a-point
  // navigator (motion-api.md S3.3); it was never a drivetrain limit.
  static constexpr float kTurnFirstAngle = 0.8726646f;

  static constexpr int kSettleMaxSteps = 12;          // [steps]
  static constexpr float kSettleRestCountsPerS = 25.0f;  // [counts/s] ~2 mm/s

  // [counts] the yaw axis must move this far, either way, before
  // Segment::wrongWay() is trusted -- a cold wheel's start-up skew reads
  // backward before real rotation begins.
  static constexpr float kMinYawProgressBeforeWrongWay = 40.0f;

  // A plain aggregate: no default member initializers, so it stays C++11.
  struct AxisLimits {
    float floor;  // [mm/s]
    float cap;    // [mm/s]
  };

  // wheelsV()'s target, slewed toward through shaper_ every tick. Exactly
  // one of seg_/hold_ is ever live.
  struct Hold {
    bool active = false;
    float v = 0.0f;         // [mm/s] target mean velocity
    float twist = 0.0f;     // [mm/s] target half-differential
    float dominant = 0.0f;  // [mm/s] max(|v-twist|, |v+twist|)
    uint32_t until = 0;     // [ms] the caller's duration deadline
  };

  // beginNudge()'s target, ticked by serviceNudge(). Exactly one of
  // seg_/hold_/nudge_ is ever live (cancelMove() clears all three).
  struct Nudge {
    bool active = false;
    float distTarget = 0.0f;  // [counts] signed mean-axis target
    float yawTarget = 0.0f;   // [counts] signed half-differential target
    // Origin captured SYNCHRONOUSLY at beginNudge() (unlike Segment's
    // lazy originPending capture): beginNudge() stages no command of
    // its own for a later service() tick to deliver -- the kernel's
    // Output is already valid to read the instant cancelMove() returns.
    float posLeft0 = 0.0f, posRight0 = 0.0f;  // [counts]
    uint32_t deadline = 0;    // [ms] the caller's timeout backstop
    int32_t pulsesFired = 0;  // [pulses] counted against kMaxNudgePulses
    // [counts] magnitude of the MOST RECENTLY fired pulse that
    // corrected EACH axis, tracked separately since a combined
    // (distance AND rotation both nonzero) nudge can correct either
    // axis on a given pulse -- see serviceNudge()'s own comment. 0
    // means "no estimate yet on this axis" (always fire while that
    // axis's remaining error exceeds its margin). This is what
    // serviceNudge()'s "stop within one step" check reads rather than a
    // caller-supplied expected step, since the per-pulse magnitude is
    // empirical (surface/robot dependent) and this engine has no other
    // way to know it.
    float lastDistStep = 0.0f;  // [counts]
    float lastYawStep = 0.0f;   // [counts]
    // [ms] serviceNudge() will not fire before this wall-clock time --
    // nudgeSettle()'s own gate, stamped after every fire; 0 at
    // beginNudge() so the FIRST pulse is never delayed by it.
    uint32_t readyAt = 0;
  };

  uint32_t now() const;  // [ms]

  // The shared rest test settleToRest() and serviceNudge() both use --
  // mirrors the existing rest test rather than inventing a second
  // definition of "stopped".
  bool atRest(const DiffDrive::DifferentialDrive::Output& out) const;

  // The shared tail of pulseWheels() and serviceNudge(): fires
  // `widthTicks` duty ticks via the kernel's own driveDuty(), hard-
  // zeros, then settles (settleToRest()) -- see pulseWheels()'s own
  // header comment for the full contract (synchronous, drives its own
  // kernel.step() loop, no tick-engine cadence involved). Returns the
  // per-wheel encoder-count delta measured across just this call.
  PulseResult firePulseAndSettle(float ampLeft, float ampRight,
                                 int32_t widthTicks);  // [%] [%] [ticks]

  // nudge_'s own per-tick advance, dispatched from service() exactly the
  // way the Segment/Hold branches already are -- see this file's own
  // Nudge struct and beginNudge()'s header comment for the algorithm.
  bool serviceNudge();

  // Converts this segment's axis into the dominant-wheel floor/cap the
  // shaper wants. A pure turn uses the omega floor/ceiling; a straight leg
  // or blended arc uses vFloor and no cap.
  AxisLimits axisLimits(const Segment& seg) const;

  // Builds seg_ -- the shared tail of wheelsX()'s per-wheel reduction and
  // moveX()'s distance/rotation reduction. Stages no drive() call: the
  // first real command is issued by the first service(). A zero-magnitude
  // command or non-positive cruise leaves seg_ inactive and neutrals.
  void beginSegment(float distTarget, float yawTarget, float cruise,
                    uint32_t deadline);  // [counts] [counts] [mm/s] [ms]

  // Pivot now, then travel straight once that pivot completes cleanly --
  // the shared tail of moveX()'s split and goToR()'s. `deadline` spans
  // both phases.
  void queuePivotThenStraight(float pivotRotation, float straightDistance,
                              float cruise,
                              uint32_t deadline);  // [rad] [mm] [mm/s] [ms]

  // service()'s phase 1 -> phase 2 handoff: captures the pending fields
  // before beginSegment() resets seg_, then starts phase 2 normally.
  void beginPendingStraightPhase();

  // Clears BOTH seg_ and hold_ without touching the kernel -- every
  // primitive's "clear the planner" step, and endMove()'s tail.
  void cancelMove();

  DiffDrive::DifferentialDrive& kernel_;
  const DiffDrive::Clock& clock_;

  // Geometry defaults are the measured tovez/vevov bake. The measurements
  // and their derivations are in DESIGN.md; generic kits recalibrate
  // through setGeometry() (shims.cpp).
  float travelCalib_ = 0.7878f;   // [mm/deg] wheel travel per shaft degree
  float trackWidth_ = 114.2f;     // [mm] tape-measured
  float rotationalSlip_ = 0.952f; // [1] physical/odometric rotation ratio

  Segment seg_;
  Hold hold_;
  Nudge nudge_;
  VelocityShaper shaper_;
  MotionLimits limits_;

  // [pulses] the nudge stepper's own runaway backstop -- see
  // nudgeMaxPulses()'s own comment.
  static constexpr int32_t kMaxNudgePulses = 40;

  // Nudge config, read fresh at fire time by serviceNudge(). Defaults
  // and their citations live on the public getters/setters above.
  float nudgeAmplitude_ = 15.0f;    // [%]
  int32_t nudgeWidthTicks_ = 2;     // [ticks]
  float nudgeSettle_ = 0.0f;      // [ms]

  // [ms] the previous service() tick, for the shaper's dt. Re-stamped at
  // every genuine command start so the first tick's dt runs from when the
  // command was armed.
  uint32_t lastTick_ = 0;

  // Moves aborted for rotating away from the commanded direction.
  uint32_t wrongWayCount_ = 0;

  bool lastSegmentEndedByDeadline_ = false;

  bool stallReported_ = false;
};

}  // namespace diffDrive
