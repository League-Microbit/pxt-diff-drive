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

  // [rad] the |rotation| at which moveX() splits into pivot-then-straight.
  // Exposed so a caller mirroring that decision reads it rather than
  // re-typing the constant.
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

  // Supersedes any in-flight command. Splits into pivot-then-straight above
  // kTurnFirstAngle when there is also translation.
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

  // Steps the kernel until both wheels MEASURE at rest, bounded. Needed
  // because neutral() only stages a zero command and the delivering step's
  // own encoder read can otherwise freeze a nonzero velocity forever.
  // Issues no command of its own and folds nothing into odometry.
  void settleToRest();

  // The one settable shaping surface. This engine holds no shaping knob.
  MotionLimits& limits() { return limits_; }
  const MotionLimits& limits() const { return limits_; }

 private:
  // [rad] 50 deg. At or above this, pivot first, then travel straight.
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

  uint32_t now() const;  // [ms]

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
  VelocityShaper shaper_;
  MotionLimits limits_;

  // [ms] the previous service() tick, for the shaper's dt. Re-stamped at
  // every genuine command start so the first tick's dt runs from when the
  // command was armed.
  uint32_t lastTick_ = 0;

  // Moves aborted for rotating away from the commanded direction.
  uint32_t wrongWayCount_ = 0;

  bool lastSegmentEndedByDeadline_ = false;
};

}  // namespace diffDrive
