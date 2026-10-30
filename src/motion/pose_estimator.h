// diffDrive::PoseEstimator -- the robot's pose from whatever sensors it has,
// and the PoseSource goToW() reads.
//
// Encoders always integrate (through an Odometry used only to difference
// them). A gyro, when present, is the heading reference; its rate bias is
// learned (GyroBiasTracker) and removed. An optical source, when present,
// supplies position. Losing a sensor drops to the next configuration with
// the output carried over, so nothing steps.
//
// Still means the wheels are still. With an optical source the sensor's raw
// track (lever arm not applied) must be still too: a diff-drive robot cannot
// slide sideways under its own power, so a still-wheeled robot whose sensor
// moves was bumped or lifted. Then the gyro change is real rotation and goes
// to the output, and the position follows the optical track. A gyro rate
// well above any plausible bias says the same with no optical track.
//
// In motion the gyro outranks the encoders: a change they disagree with is
// rotation they missed (slip) or saw early (the gyro lags at pivot ends),
// so it is taken. Only a read faster than the robot can turn is a glitch;
// that read's excess over the encoders is skipped, the output follows the
// encoders until the jump holds steady, and then the heading is the gyro's
// without it. The encoders never become the reference.
#pragma once

#include <cmath>
#include <cstdint>

#include "../core/heading_wrap.h"
#include "../platform/gyro_source.h"
#include "../platform/sensor_config.h"
#include "gyro_bias_tracker.h"
#include "odometry.h"

namespace diffDrive {

struct PoseEstimatorParams {
  // Encoders must stay unchanged this long before the robot counts as
  // still (the chassis keeps turning briefly after the wheels stop).
  float restDwell = 0.25f;  // [s]

  // A gyro change is believed when it agrees with the encoders' to within
  // slack + fraction of theirs.
  float gateSlack = 0.0174533f;  // [rad] 1 deg
  float gateFraction = 0.3f;     // [1]
  int confirmSamples = 3;        // consecutive consistent samples that make a
                                 // rejected jump the new reference

  // No read of a turning robot exceeds this. Twice the 120 deg/s the closed
  // profile turns at (the MotionLimits omegaMax default of the fleet); a
  // slower-reading or faster-turning robot must move it.
  float maxPlausibleRate = 4.18879f;  // [rad/s] 240 deg/s

  // Disturbance gate, over a window the wheels are still. The raw sensor
  // track moved at most 1.2 mm in any window up to 200 s on all four fleet
  // robots at rest (TLM 1 mm resolution; arm-corrected centre creep removed;
  // captures/otos-gyro-fleet-20261028/{gopiv,vevov,tovez,zuvat}.jsonl), so
  // 5 mm is 4x that floor and still well under a hand bump. The gyro rate
  // limit is 3 deg/s: 30x the bias ceiling `bias.maxRate`, 70x the largest
  // bias seen (150 deg/h, vevov after a power cycle,
  // captures/vevov-otos-drift-20261028/gyro3-bench-afterboot.jsonl), and
  // over 10 sd of the white rate noise the bias tracker's 0.3 deg heading
  // random walk implies at a 96 ms read; a hand turn is faster.
  float disturbDistance = 5.0f;     // [mm] raw sensor track within one window
  float disturbRate = 0.0523599f;   // [rad/s] gyro rate beyond bias
  float disturbWindow = 1.0f;       // [s] evidence window; a disturbance lasts
                                    // this long past the last evidence

  int dropoutReads = 8;   // consecutive failed or stale reads that drop a sensor
  int recoverReads = 10;  // consecutive good reads that bring it back

  // Gyro frozen: identical readings while the encoders turn.
  float freezeStep = 0.00175f;  // [rad] 0.1 deg of encoder turn per read
  float freezeTurn = 0.0524f;   // [rad] 3 deg total

  // Optical increments are believed within slack + fraction of the encoder
  // distance covered over the same interval.
  float opticalSlack = 30.0f;     // [mm]
  float opticalFraction = 0.5f;   // [1]
  float opticalFreezeDistance = 40.0f;  // [mm] encoder distance the optical source
                                        // fell short by, beyond a 20% allowance
  int opticalRejectLimit = 8;     // consecutive rejected takes that drop it
  float opticalScale = 1.0f;      // [1] linear scale applied to increments

  GyroBiasParams bias;
};

class PoseEstimator : public PoseSource {
 public:
  using Output = DiffDrive::DifferentialDrive::Output;

  explicit PoseEstimator(const MotionEngine& engine,
                         const PoseEstimatorParams& params = PoseEstimatorParams())
      : params_(params), enc_(engine), bias_(params.bias) {}

  // Sensors are borrowed and must outlive the estimator. `config` is what
  // the boot probe found; a sensor the config does not name is ignored.
  void attachSensors(GyroSource* gyro, OpticalPositionSource* optical,
                     SensorConfig config) {
    gyro_ = config == SensorConfig::kEncodersOnly ? nullptr : gyro;
    optical_ = config == SensorConfig::kOptical ? optical : nullptr;
    gyroUp_ = gyro_ != nullptr;
    opticalUp_ = optical_ != nullptr;
    gyroPrimed_ = false;
    awaitBaseline_ = true;
  }

  // One tick. The encoders are folded in every call; the sensors are read
  // only when `sampleSensors`, so the caller sets the read cadence.
  void update(const Output& out, float dt, bool sampleSensors = true) {
    enc_.update(out);  // [s]
    const float dCentre = enc_.stepCentre();    // [mm]
    const float dHeading = enc_.stepHeading();  // [rad]
    restTime_ = (dCentre == 0.0f && dHeading == 0.0f) ? restTime_ + dt : 0.0f;
    time_ += dt;
    encoderTurn_ += dHeading;
    sinceAnchorTime_ += dt;
    sinceAnchorTurn_ += dHeading;
    sinceReadTurn_ += dHeading;
    stillTurn_ += std::fabs(dHeading);
    if (!gyroUp_) gyroTurn_ += dHeading;

    const float mid = heading_ + 0.5f * dHeading;  // [rad]
    const float stepX = dCentre * std::cos(mid);   // [mm]
    const float stepY = dCentre * std::sin(mid);   // [mm]
    x_ += stepX;
    y_ += stepY;
    heading_ += dHeading;
    pendingX_ += stepX;
    pendingY_ += stepY;
    pendingPath_ += std::fabs(dCentre);

    if (sampleSensors && gyro_ != nullptr) sampleGyro();
  }

  // Same "update() first" idiom as Odometry::seed(). The learned bias is
  // kept: it belongs to the gyro, not to the frame.
  void seed(float x, float y, float heading) {  // [mm] [mm] [rad]
    x_ = baseX_ = x;
    y_ = baseY_ = y;
    heading_ = anchorHeading_ = heading;
    sinceAnchorTime_ = sinceAnchorTurn_ = 0.0f;
    pendingX_ = pendingY_ = pendingPath_ = 0.0f;
    holdX_ = holdY_ = 0.0f;
    agree_ = 0;
    skipped_ = 0.0f;
  }
  void reset() { seed(0.0f, 0.0f, 0.0f); }

  // The optical source was re-seeded: take the next gyro reading as the new
  // reference instead of reading the jump as a step, and the next optical
  // take as a baseline that carries no motion.
  void reprime() {
    gyroPrimed_ = false;
    awaitBaseline_ = true;
  }

  float x() const override { return x_; }              // [mm] world frame
  float y() const override { return y_; }              // [mm] world frame
  float heading() const override { return heading_; }  // [rad] unwrapped
  bool hasHeadingReference() const override { return gyroUp_; }
  bool tracksMotion() const override { return true; }

  SensorConfig config() const {
    if (!gyroUp_) return SensorConfig::kEncodersOnly;
    return opticalUp_ ? SensorConfig::kOptical : SensorConfig::kGyroOnly;
  }
  bool atRest() const { return restTime_ >= params_.restDwell; }

  float biasRate() const { return bias_.rate(); }  // [rad/s]
  const GyroBiasTracker& bias() const { return bias_; }
  uint32_t rejectedCount() const { return rejected_; }  // gyro + optical
  uint32_t stepCount() const { return steps_; }  // jumps taken as the new reference
  uint32_t disturbCount() const { return disturbs_; }  // bumps and lifts seen
  bool disturbed() const { return time_ < disturbedUntil_; }
  const PoseEstimatorParams& params() const { return params_; }

 private:
  struct OpticalTake {
    bool got = false;
    float dx = 0.0f, dy = 0.0f;        // [mm] centre
    float rawDx = 0.0f, rawDy = 0.0f;  // [mm] sensor track
  };

  bool consistent(float change, float expected) const {  // [rad] [rad]
    return std::fabs(change - expected) <=
           params_.gateSlack + params_.gateFraction * std::fabs(expected);
  }

  void dropGyro() {
    gyroTurn_ += sinceAnchorTurn_;
    gyroUp_ = false;
    opticalUp_ = false;
    recovered_ = 0;
  }

  void sampleGyro() {
    const GyroReadStatus status = gyro_->readGyro();
    if (status != GyroReadStatus::kOk) {
      recovered_ = 0;
      if (++silent_ >= params_.dropoutReads && gyroUp_) dropGyro();
      return;
    }
    silent_ = 0;
    const float raw = gyro_->gyroHeading();  // [rad]
    const float rate = gyro_->gyroRate();    // [rad/s]
    const bool unchanged = gyroPrimed_ && raw == previousRaw_ && rate == previousRate_;
    if (!unchanged) stillTurn_ = 0.0f;

    if (!gyroPrimed_) {
      gyroPrimed_ = true;
      anchor(raw);
      previousRaw_ = raw;
      previousRate_ = rate;
      previousTime_ = time_;
      return;
    }
    if (!gyroUp_) {
      if (unchanged && frozenDown_) {
        recovered_ = 0;
      } else {
        frozenDown_ = false;
        if (++recovered_ >= params_.recoverReads) {
          gyroUp_ = true;
          opticalUp_ = optical_ != nullptr;
          anchor(raw);
          baseX_ = x_;
          baseY_ = y_;
          pendingX_ = pendingY_ = pendingPath_ = 0.0f;
        }
      }
      previousRaw_ = raw;
      previousRate_ = rate;
      previousTime_ = time_;
      sinceReadTurn_ = 0.0f;
      return;
    }
    if (unchanged && stillTurn_ > params_.freezeStep) {
      if (stillTurn_ > params_.freezeTurn) {
        frozenDown_ = true;
        dropGyro();
      }
      return;
    }

    OpticalTake take;
    if (optical_ != nullptr) readOptical(take);

    const bool still = atRest() && sinceAnchorTurn_ == 0.0f;
    if (still) {
      watchRest(rate, take);
    } else {
      watching_ = false;
    }
    const float change = wrapRadians(raw - anchorRaw_);  // [rad]
    if (!still) watchGlitch(raw);
    if (still && !disturbed()) {
      holdAtRest(change);
    } else if (still) {
      followDisturbance(change);
    } else if (consistent(change, sinceAnchorTurn_) || skipped_ == 0.0f) {
      acceptMoving(change);
    } else {
      rejectJump(raw);
    }
    previousRaw_ = raw;
    previousRate_ = rate;
    previousTime_ = time_;
    sinceReadTurn_ = 0.0f;
    if (optical_ != nullptr) sampleOptical(raw, take, still);
  }

  void watchGlitch(float raw) {  // [rad]
    const float sinceRead = wrapRadians(raw - previousRaw_);  // [rad]
    const float readTime = time_ - previousTime_;             // [s]
    if (std::fabs(sinceRead) >
        params_.maxPlausibleRate * readTime + params_.gateSlack) {
      skipped_ += sinceRead - sinceReadTurn_;
    }
  }

  void anchor(float raw) {
    anchorRaw_ = raw;
    anchorHeading_ = heading_;
    sinceAnchorTime_ = sinceAnchorTurn_ = 0.0f;
    agree_ = 0;
    skipped_ = 0.0f;
  }

  // Wheels and sensor both still: collect evidence of a bump or lift.
  void watchRest(float rate, const OpticalTake& take) {  // [rad/s]
    if (!watching_) {
      watching_ = true;
      watchStart_ = time_;
      watchX_ = watchY_ = 0.0f;
    }
    bool evidence = false;
    if (opticalUp_) {
      watchX_ += take.rawDx;
      watchY_ += take.rawDy;
      evidence = std::hypot(watchX_, watchY_) > params_.disturbDistance;
    }
    evidence = evidence ||
               std::fabs(rate - bias_.rate()) > params_.disturbRate;
    if (evidence) {
      if (!disturbed()) ++disturbs_;
      disturbedUntil_ = time_ + params_.disturbWindow;
    }
    if (evidence || time_ - watchStart_ >= params_.disturbWindow) {
      watchStart_ = time_;
      watchX_ = watchY_ = 0.0f;
      if (!evidence && !disturbed()) holdX_ = holdY_ = 0.0f;
    }
  }

  // The wheels say still and the gyro says otherwise: the robot was moved
  // by hand, so the change is real rotation. It is kept out of the bias
  // fit on both sides, as a step is.
  void followDisturbance(float change) {  // [rad]
    heading_ = anchorHeading_ + change - bias_.rate() * sinceAnchorTime_;
    gyroTurn_ += sinceAnchorTurn_;
    anchorRaw_ = wrapRadians(anchorRaw_ + change);
    anchorHeading_ = heading_;
    sinceAnchorTime_ = sinceAnchorTurn_ = 0.0f;
    agree_ = 0;
    skipped_ = 0.0f;
  }

  // Still: the true rate is zero, so the whole change is bias (or a step).
  // The heading is held and the change is not folded into the output.
  void holdAtRest(float change) {  // [rad]
    const float plausible = bias_.params().maxRate * sinceAnchorTime_ +
                            params_.gateSlack;  // [rad]
    if (std::fabs(change) <= plausible) {
      bias_.observeRest(change, sinceAnchorTime_);
      gyroTurn_ += change;
    } else {
      ++steps_;
      gyroTurn_ += sinceAnchorTurn_;
    }
    anchorRaw_ = wrapRadians(anchorRaw_ + change);
    anchorHeading_ = heading_;
    sinceAnchorTime_ = sinceAnchorTurn_ = 0.0f;
    agree_ = 0;
    skipped_ = 0.0f;
  }

  void acceptMoving(float change, float glitch = 0.0f) {  // [rad] [rad]
    const float turn = change - glitch;  // [rad]
    heading_ = anchorHeading_ + turn - bias_.rate() * sinceAnchorTime_;
    gyroTurn_ += turn;
    anchorRaw_ = wrapRadians(anchorRaw_ + change);
    anchorHeading_ = heading_;
    sinceAnchorTime_ = sinceAnchorTurn_ = 0.0f;
    agree_ = 0;
    skipped_ = 0.0f;
    bias_.observeMotion(time_, gyroTurn_, encoderTurn_);
  }

  // The output stays on the encoders. Only a jump that then holds steady
  // for confirmSamples becomes the new reference; the heading then comes
  // from the gyro since the anchor with the glitch taken out.
  void rejectJump(float raw) {  // [rad]
    ++rejected_;
    const float sinceRead = wrapRadians(raw - previousRaw_);  // [rad]
    agree_ = consistent(sinceRead, sinceReadTurn_) ? agree_ + 1 : 0;
    if (agree_ < params_.confirmSamples) return;
    ++steps_;
    acceptMoving(wrapRadians(raw - anchorRaw_), skipped_);
  }

  void readOptical(OpticalTake& take) {
    take.got = optical_->takeOpticalIncrement(take.dx, take.dy, take.rawDx,
                                              take.rawDy);
    if (take.got) {
      noFix_ = 0;
      awaitBaseline_ = false;
    } else if (awaitBaseline_) {
      awaitBaseline_ = false;
      baseX_ = x_;
      baseY_ = y_;
      pendingX_ = pendingY_ = pendingPath_ = 0.0f;
    } else {
      recovered_ = 0;
      if (++noFix_ >= params_.dropoutReads && opticalUp_) opticalUp_ = false;
    }
  }

  void sampleOptical(float raw, const OpticalTake& take, bool still) {  // [rad]
    if (!take.got) return;
    const float dx = take.dx;  // [mm]
    const float dy = take.dy;  // [mm]
    const bool follow = still && disturbed();
    if (still && !follow) {
      const float turn = wrapRadians(heading_ - raw);  // [rad]
      holdX_ += params_.opticalScale * (std::cos(turn) * dx - std::sin(turn) * dy);
      holdY_ += params_.opticalScale * (std::sin(turn) * dx + std::cos(turn) * dy);
      baseX_ = x_;
      baseY_ = y_;
      pendingX_ = pendingY_ = pendingPath_ = 0.0f;
      return;
    }
    const float turn = wrapRadians(heading_ - raw);  // [rad]
    const float c = std::cos(turn);
    const float s = std::sin(turn);
    const float scale = params_.opticalScale;
    const float ix = scale * (c * dx - s * dy) + holdX_;  // [mm]
    const float iy = scale * (s * dx + c * dy) + holdY_;  // [mm]
    holdX_ = holdY_ = 0.0f;
    if (follow) {
      if (opticalUp_) {
        baseX_ += ix;
        baseY_ += iy;
        rejectRun_ = 0;
        x_ = baseX_;
        y_ = baseY_;
      } else {
        baseX_ = x_;
        baseY_ = y_;
      }
      pendingX_ = pendingY_ = pendingPath_ = 0.0f;
      return;
    }

    deficit_ = std::fmax(0.0f, deficit_ +
                                   (1.0f - kOpticalAllowance) *
                                       std::hypot(pendingX_, pendingY_) -
                                   std::hypot(ix, iy));
    if (deficit_ > params_.opticalFreezeDistance && opticalUp_) {
      opticalUp_ = false;
      recovered_ = 0;
    }
    const float error = std::hypot(ix - pendingX_, iy - pendingY_);  // [mm]
    const bool agrees =
        error <= params_.opticalSlack + params_.opticalFraction * pendingPath_;
    if (opticalUp_) {
      if (agrees) {
        baseX_ += ix;
        baseY_ += iy;
        rejectRun_ = 0;
      } else {
        ++rejected_;
        baseX_ += pendingX_;
        baseY_ += pendingY_;
        if (++rejectRun_ >= params_.opticalRejectLimit) {
          opticalUp_ = false;
          recovered_ = 0;
        }
      }
      x_ = baseX_;
      y_ = baseY_;
    } else {
      recovered_ = (agrees && deficit_ == 0.0f) ? recovered_ + 1 : 0;
      if (recovered_ >= params_.recoverReads) {
        opticalUp_ = true;
        rejectRun_ = 0;
      }
      baseX_ = x_;
      baseY_ = y_;
    }
    pendingX_ = pendingY_ = pendingPath_ = 0.0f;
  }

  static constexpr float kOpticalAllowance = 0.2f;  // [1]

  PoseEstimatorParams params_;
  Odometry enc_;
  GyroBiasTracker bias_;
  GyroSource* gyro_ = nullptr;
  OpticalPositionSource* optical_ = nullptr;
  bool gyroUp_ = false, opticalUp_ = false;
  bool gyroPrimed_ = false, frozenDown_ = false;
  bool awaitBaseline_ = false;  // the next take only sets the source's baseline

  float x_ = 0.0f, y_ = 0.0f;  // [mm]
  float heading_ = 0.0f;       // [rad]
  float time_ = 0.0f;          // [s]
  float restTime_ = 0.0f;      // [s]
  float encoderTurn_ = 0.0f;   // [rad] cumulative
  float gyroTurn_ = 0.0f;      // [rad] cumulative, steps excluded

  float anchorRaw_ = 0.0f;       // [rad] last gyro reading folded in
  float anchorHeading_ = 0.0f;   // [rad] output at that reading
  float sinceAnchorTime_ = 0.0f; // [s]
  float sinceAnchorTurn_ = 0.0f; // [rad] encoder turn
  float previousRaw_ = 0.0f;     // [rad] last fresh reading, folded in or not
  float previousRate_ = 0.0f;    // [rad/s]
  float previousTime_ = 0.0f;    // [s]
  float skipped_ = 0.0f;         // [rad] glitch excess since the anchor
  float sinceReadTurn_ = 0.0f;   // [rad] encoder turn since that reading
  float stillTurn_ = 0.0f;       // [rad] encoder turn while readings repeat

  float baseX_ = 0.0f, baseY_ = 0.0f;  // [mm] output as of the last optical take
  float pendingX_ = 0.0f, pendingY_ = 0.0f;  // [mm] encoder motion since then
  float pendingPath_ = 0.0f;                 // [mm]
  float deficit_ = 0.0f;                     // [mm]
  float holdX_ = 0.0f, holdY_ = 0.0f;        // [mm] optical motion seen while still,
                                             // applied if the wheels start moving

  bool watching_ = false;
  float watchStart_ = 0.0f;                  // [s]
  float watchX_ = 0.0f, watchY_ = 0.0f;      // [mm] raw track since then
  float disturbedUntil_ = 0.0f;              // [s]

  int silent_ = 0, noFix_ = 0, agree_ = 0, recovered_ = 0, rejectRun_ = 0;
  uint32_t rejected_ = 0, steps_ = 0, disturbs_ = 0;
};

}  // namespace diffDrive
