// gyro_bias_tracker.h -- GyroBiasTracker: the gyro's slowly varying rate
// bias, learned from two observations. PoseEstimator owns one and decides
// which observation applies; this class holds only the model.
//
//   rest    the robot is still, so every gyro change is bias (ZARU)
//   motion  least-squares fit of (gyro - encoder heading) on
//           [1, time, encoder net turn]; the third term soaks up the
//           encoders' own scale error
//
// The estimate is the inverse-variance blend of the two. There is no
// stored per-robot value: it starts at zero and never stops adapting.
#pragma once

#include <cmath>

namespace diffDrive {

struct GyroBiasParams {
  // A rest window shorter than this is accumulated, not used.
  float restWindow = 2.0f;  // [s]

  // Rest observations older than this are forgotten. The measured bias
  // ramps by about 1-1.5 deg/h per minute for the first half hour after
  // boot (captures/otos-gyro-fleet-20261028/); a 3 min average lags that
  // ramp by about 4 deg/h and averages out about 13 deg/h of noise, both
  // under a tenth of a degree over a 30 s move.
  float timeConstant = 180.0f;  // [s]

  // A rest window whose mean rate exceeds this is a step or a push, not
  // bias; the largest bias seen was about 150 deg/h (vevov after a
  // power cycle, captures/vevov-otos-drift-20261028/gyro3-bench-afterboot.jsonl),
  // so this leaves 2.4x margin.
  float maxRate = 0.0017453f;  // [rad/s] 0.1 deg/s

  // Heading noise as a random walk. The second difference of the idle
  // heading gives 0.0032-0.0057 deg/sqrt(s) on the four fleet robots
  // (tests/host/fixtures/pose_estimator/fleet-*.jsonl); this is twice the
  // worst, 0.012 deg/sqrt(s).
  float noiseDensity = 0.000209f;  // [rad/sqrt(s)]

  // The motion fit treats encoder slip, which is correlated in time, as
  // white noise, so its variance is far too small. Its weight is held to
  // (moved / restStale)^2 times the rest weight, where `moved` is the time
  // spent driving since the last rest observation: rest evidence leads
  // while it is fresh, and the fit takes over only after a drive this long
  // (the weights are equal at 300 s, 4:1 at 600 s). With no rest evidence
  // at all the fit is unlimited.
  float restStale = 300.0f;  // [s]

  // Motion fit: a heading sample is taken at most this often, the fit
  // forgets over this long, and its blend weight is scaled down because
  // its residuals are not white.
  float motionInterval = 0.25f;  // [s]
  float motionForgetting = 600.0f;  // [s]
  float motionTrust = 0.3f;  // [1]
};

class GyroBiasTracker {
 public:
  explicit GyroBiasTracker(const GyroBiasParams& params = GyroBiasParams())
      : params_(params) {
    reset();
  }

  void reset() {
    restTime_ = restChange_ = 0.0f;
    restRate_ = 0.0f;
    restLearned_ = 0.0f;
    motionSamples_ = 0;
    movedSinceRest_ = 0.0f;
    lastMotionTime_ = -1.0f;
    residual_ = kResidualFloor;
    for (int i = 0; i < 3; ++i) {
      theta_[i] = 0.0f;
      for (int j = 0; j < 3; ++j) p_[i][j] = (i == j) ? kInitialCovariance : 0.0f;
    }
  }

  // The robot was still for dt; the gyro heading changed by `change`.
  void observeRest(float change, float dt) {  // [rad] [s]
    movedSinceRest_ = 0.0f;
    restTime_ += dt;
    restChange_ += change;
    if (restTime_ < params_.restWindow) return;
    const float observed = restChange_ / restTime_;  // [rad/s]
    const float span = restTime_;                    // [s]
    restTime_ = restChange_ = 0.0f;
    if (std::fabs(observed) > params_.maxRate) return;
    restLearned_ += span;
    const float memory = std::fmin(restLearned_, params_.timeConstant);  // [s]
    restRate_ += (span / memory) * (observed - restRate_);
  }

  // Cumulative since the estimator started, only while the robot moves.
  void observeMotion(float time, float gyroTurn, float encoderTurn) {
    // [s] [rad] [rad]
    if (lastMotionTime_ >= 0.0f && time - lastMotionTime_ < params_.motionInterval) {
      return;
    }
    const float gap = lastMotionTime_ < 0.0f ? 0.0f : time - lastMotionTime_;  // [s]
    lastMotionTime_ = time;
    movedSinceRest_ += std::fmin(gap, kMovedGapLimit);
    const float forget = std::exp(-gap / params_.motionForgetting);
    const float phi[3] = {1.0f, time * kTimeScale, encoderTurn};
    const float y = gyroTurn - encoderTurn;

    float pPhi[3];
    float denom = forget;
    for (int i = 0; i < 3; ++i) {
      pPhi[i] = p_[i][0] * phi[0] + p_[i][1] * phi[1] + p_[i][2] * phi[2];
      denom += phi[i] * pPhi[i];
    }
    const float error = y - (theta_[0] * phi[0] + theta_[1] * phi[1] +
                             theta_[2] * phi[2]);
    for (int i = 0; i < 3; ++i) theta_[i] += pPhi[i] / denom * error;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        p_[i][j] = (p_[i][j] - pPhi[i] * pPhi[j] / denom) / forget;
      }
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = i + 1; j < 3; ++j) {
        const float mean = 0.5f * (p_[i][j] + p_[j][i]);
        p_[i][j] = p_[j][i] = mean;
      }
    }
    residual_ += 0.05f * (error * error - residual_);
    if (residual_ < kResidualFloor) residual_ = kResidualFloor;
    if (motionSamples_ < kMotionWarmup) ++motionSamples_;
  }

  float rate() const {  // [rad/s]
    const float restWeight = restWeightNow();
    const float motionWeight = motionWeightNow();
    const float total = restWeight + motionWeight;
    if (total <= 0.0f) return 0.0f;
    return (restWeight * restRate_ + motionWeight * motionRateNow()) / total;
  }

  float restRate() const { return restRate_; }          // [rad/s]
  float motionRate() const { return motionRateNow(); }  // [rad/s]
  float restLearned() const { return restLearned_; }    // [s]
  const GyroBiasParams& params() const { return params_; }

 private:
  static constexpr float kTimeScale = 1.0f / 600.0f;  // [1/s]
  static constexpr float kInitialCovariance = 1000.0f;
  static constexpr float kResidualFloor = 4.0e-6f;  // [rad^2] (0.11 deg)^2
  static constexpr int kMotionWarmup = 12;
  static constexpr float kMovedGapLimit = 1.0f;  // [s] longer gaps were idle

  float restWeightNow() const {  // [s^2/rad^2]
    const float memory = std::fmin(restLearned_, params_.timeConstant);
    return memory / (params_.noiseDensity * params_.noiseDensity);
  }
  float motionRateNow() const { return theta_[1] * kTimeScale; }  // [rad/s]
  float motionWeightNow() const {
    if (motionSamples_ < kMotionWarmup) return 0.0f;
    const float variance =
        p_[1][1] * residual_ * kTimeScale * kTimeScale;  // [rad^2/s^2]
    if (!(variance > 0.0f)) return 0.0f;
    const float weight = params_.motionTrust / variance;
    if (restLearned_ <= 0.0f) return weight;
    const float fresh = movedSinceRest_ / params_.restStale;  // [1]
    return std::fmin(weight, restWeightNow() * fresh * fresh);
  }

  GyroBiasParams params_;
  float restTime_ = 0.0f, restChange_ = 0.0f;  // [s] [rad]
  float restRate_ = 0.0f;                      // [rad/s]
  float restLearned_ = 0.0f;                   // [s]
  float lastMotionTime_ = -1.0f;               // [s]
  float movedSinceRest_ = 0.0f;                // [s]
  int motionSamples_ = 0;
  float residual_ = kResidualFloor;            // [rad^2]
  float theta_[3];
  float p_[3][3];
};

}  // namespace diffDrive
