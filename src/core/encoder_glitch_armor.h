// encoder_glitch_armor.h -- EncoderGlitchArmor: the raw-counts
// plausibility decision extracted from NezhaMotorPort::collect()
// (sprint 006 ticket 005, clasi/issues/brick-reset-odometry-teleport.md
// / code review R-07, KERN-07).
//
// nezha_port.h includes pxt.h unconditionally (src/DESIGN.md S1's
// layering table), so NezhaMotorPort itself cannot be compiled into any
// host test -- there is no existing seam that exercises its I2C-bound
// methods host-side, and nothing in this sprint changes that. This
// header carries the one piece of the fix that CAN be host-compiled and
// host-tested directly (tests/host/test_encoder_glitch_armor.py): the
// pure two-strike plausibility decision. Wiring this into
// NezhaMotorPort::collect() itself is review-verified only -- see that
// method's own comment in nezha_port.cpp.
//
// No project includes, no pxt.h -- <cstdint> only, so this stays
// host-portable (src/DESIGN.md S1) and is covered by
// tests/host/test_cxx11_syntax_gate.py via a dedicated syntax-check
// translation unit (tests/host/encoder_glitch_armor_syntax_check.cpp --
// encoder_glitch_armor.h has no natural .cpp of its own).
//
// **What this fixes.** The pre-existing two-strike rule rejects an
// implausible raw-counts jump on its first appearance, then ACCEPTS it
// as truth if a second, mutually-consistent reading follows (the
// documented hand-rotation re-sync path: a hand-repositioned wheel
// reads a real, self-consistent jump). That rule cannot tell "the wheel
// really moved that far between reads" apart from "the counter itself
// restarted" (e.g. a brick MCU reset/brownout, encOffset_ captured once
// at begin() and never re-baselined) -- both look identical: implausible
// first read, then a second read consistent with the first. This class
// adds the missing THIRD outcome for that same trigger:
// kAcceptAsRebaseline, which the caller turns into an offset re-anchor
// instead of an integrated jump, so a genuine reset stops teleporting
// odometry (measured ~4 m at a typical ~50k-count reset, R-07) without
// touching the existing behavior for the other two outcomes (an
// ordinary plausible reading, or a first implausible reading with no
// consistent second one yet).
#pragma once

#include <cstdint>

namespace diffDrive {

// A pure, stateful (but hardware-free) plausibility gate over a stream
// of raw encoder counts. No I2C, no CODAL, no floating-point clock --
// evaluate() is a function of the raw counts stream alone, called once
// per successfully-read sample.
class EncoderGlitchArmor {
 public:
  enum class Decision : uint8_t {
    kAccept,             // plausible (or not yet primed): integrate as motion.
    kAcceptAsRebaseline, // second consecutive self-consistent reading after
                         // an implausible jump: the COUNTER restarted, not
                         // the wheel -- caller re-anchors its offset instead
                         // of integrating the jump as motion.
    kRejectPending,      // first implausible reading: held, awaiting a
                         // second consistent one to disambiguate the two
                         // outcomes above. Position/velocity/sampleTime all
                         // HOLD at the caller for this outcome.
  };

  // kMaxDeltaCounts -- the plausibility bound separating "real motion in
  // one sample gap" from "the counter jumped."
  //
  // Derivation (not a tuned round number -- this codebase has been bitten
  // by exactly that before: rotationalSlip = 0.952, a constant nobody can
  // re-derive, clasi/issues/rotational-slip-not-tunable.md):
  //
  //   - Kernel cadence: DiffDrive::DifferentialDrive::Config::cyclePeriod
  //     defaults to 24 ms (diffdrive.h), and NezhaMotorPort::collect() is
  //     called exactly once per tick (nezha_port.cpp's tick()). The
  //     common-case gap between two successful collects is one cycle.
  //   - But the gap this check must tolerate can span roughly TWO cycles:
  //     an I2C read failure (connected_ = false) or a first implausible
  //     reading (rejectPending_) both hold lastGoodRaw_ unchanged for at
  //     least one extra tick before the next comparison -- code review
  //     R-07 / KERN-07's own arithmetic independently lands on "24-48 ms"
  //     for this same reason (verify-kernel.md).
  //   - Physically achievable wheel velocity: fullDutyVelocity, this
  //     kernel's own MEASURED 100%-duty wheel rate (shims.cpp:
  //     cfg.fullDutyVelocity = 10795.0f -- the same figure the kernel's
  //     own velocity-mode duty conversion already treats as ground truth
  //     for "as fast as this wheel can turn," diffdrive.cpp's
  //     dutyPerSpeed = 1/fullDutyVelocity). No commanded motion can
  //     exceed it without the shaping pipeline's [-1, 1] duty clamp
  //     already having been violated.
  //   - Max plausible delta in one 24 ms cycle:
  //       10795 counts/s * 0.024 s ~= 259 counts.
  //     Over the worst-case ~2-cycle gap: ~= 518 counts.
  //   - kMaxDeltaCounts = 5000 sits ~10x above that worst-case plausible
  //     motion delta (headroom for I2C jitter, PID overshoot, and several
  //     consecutive held ticks) while sitting ~10x below the smallest
  //     confirmed discontinuity magnitude this ticket targets (a brick
  //     reset's ~50,000-count jump, R-07). Comfortable separation on both
  //     sides, derived from measured hardware limits and the kernel's own
  //     configured cadence -- not picked to make a specific bench run
  //     pass.
  static constexpr int32_t kMaxDeltaCounts = 5000;

  // Evaluates one new raw sample against the accumulated two-strike
  // state and returns the plausibility decision. Mutates internal state
  // (lastGoodRaw_/lastRejectedRaw_/rejectPending_) exactly the way the
  // original inline logic did, for every input this class has been
  // primed with (see markPrimed()/seedLastGoodRaw() below -- before
  // priming, every reading is accepted unconditionally, matching the
  // pre-extraction behavior of an un-begun port).
  Decision evaluate(int32_t raw) {
    if (primed_) {
      const int32_t delta = raw - lastGoodRaw_;
      const int32_t mag = delta < 0 ? -delta : delta;
      if (mag > kMaxDeltaCounts) {
        const int32_t rejDelta = raw - lastRejectedRaw_;
        const int32_t rejMag = rejDelta < 0 ? -rejDelta : rejDelta;
        if (rejectPending_ && rejMag <= kMaxDeltaCounts) {
          // Second consecutive self-consistent implausible reading: the
          // counter restarted, not the wheel. Caller re-anchors instead
          // of integrating this as motion.
          rejectPending_ = false;
          lastGoodRaw_ = raw;
          return Decision::kAcceptAsRebaseline;
        }
        lastRejectedRaw_ = raw;
        rejectPending_ = true;
        return Decision::kRejectPending;
      }
    }
    rejectPending_ = false;
    lastGoodRaw_ = raw;
    return Decision::kAccept;
  }

  // The last raw count value this armor accepted as truth (either
  // outcome that returns kAccept/kAcceptAsRebaseline updates this). The
  // caller's own rebaseline() reads this the same way it always read
  // NezhaMotorPort's private lastGoodRaw_ member before this extraction.
  int32_t lastGoodRaw() const { return lastGoodRaw_; }

  // Seeds the two-strike baseline without going through evaluate() --
  // for NezhaMotorPort::begin()'s own initial median-of-3 read, which
  // is not itself a "sample stream" evaluation.
  void seedLastGoodRaw(int32_t raw) { lastGoodRaw_ = raw; }

  // Arms the plausibility check. Split from seedLastGoodRaw() because
  // the original code primed unconditionally at the end of begin() even
  // when the initial read produced no usable sample (good == 0) --
  // lastGoodRaw_ stays at its default (0) in that case, but the
  // plausibility gate still arms, exactly reproducing that corner case.
  void markPrimed() { primed_ = true; }

 private:
  int32_t lastGoodRaw_ = 0;
  int32_t lastRejectedRaw_ = 0;
  bool rejectPending_ = false;
  bool primed_ = false;
};

}  // namespace diffDrive
