// encoder_pose_source.h -- EncoderPoseSource: a second diffDrive::PoseSource
// implementation (motion_engine.h) over shims.cpp's existing dead-reckoned
// odometry, for robots with no OTOS fitted -- most of the fleet (the OTOS
// is on vevov only; tovez/gopiv/zeguz have none). Closes
// no-encoder-odometry-posesource-fallback.md (sprint 006 ticket 007,
// motion-api.md S3.6: "go_to_w's pose source is pluggable -- OTOS when
// fitted, encoder odometry otherwise"). See this sprint's design overlay
// S7/S9 for the full write-up this file implements.
//
// Same three-method shape as OtosPort (src/platform/otos_port.h): holds const
// float& references to the caller's already-computed x/y/heading and
// returns them verbatim -- it does NOT compute odometry itself.
// shims.cpp's odomUpdate() remains the only place that advances those
// fields; this class is a read-only adapter in front of them, exactly
// the way OtosPort is a read-only accessor in front of its own cached
// I2C read.
//
// LIFETIME (ticket's own AC: must be documented, not just implied). This
// class holds REFERENCES, bound once at construction -- a C++ reference
// member cannot be re-seated. It must therefore be constructed with a
// lifetime tied to Rig's own lazy-singleton, process-lifetime instance
// (shims.cpp wires it in as a Rig member, declared AFTER the x/y/heading
// fields it binds to, mirroring the exact declaration-order rule Rig's
// own header comment already states for `engine`/`kernel_`/`clock_`). A
// EncoderPoseSource constructed over anything shorter-lived than Rig --
// a local, a temporary, a per-call stack object -- would leave its
// references dangling the instant that shorter scope exits, with no
// compiler diagnostic to catch it. Do not construct one anywhere but as
// a member of the object that owns the fields it binds to.
//
// HEADING WRAP CONVENTION: deliberately UNWRAPPED, returned verbatim from
// whatever the caller's heading field holds. This is the other of the two
// contractually valid PoseSource wrap conventions motion_engine.h's own
// PoseSource comment describes -- OtosPort wraps to (-pi, pi] because the
// OTOS chip's int16 register does; this class does not, because
// shims.cpp's Rig::heading field does not (odomUpdate() accumulates it
// without ever normalizing). Both conventions are contractually valid
// because MotionEngine::goToR()/goToW() consume heading() only through
// cos()/sin() (wrap-invariant) -- do NOT "fix" this to match OtosPort's
// convention; that would violate motion-api.md S3.6's explicit
// requirement for this specific implementation. A caller that ever
// DIFFERENCES two heading() reads (rather than taking their cos/sin)
// must not assume a shared wrap convention across PoseSource
// implementations.
//
// EPOCH-GUARDED REBASELINE (motion-api.md S3.6): this class needs no
// epoch-tracking of its own. It reads the same Rig-local x/y/heading
// state shims.cpp's odomUpdate() already produces, and odomUpdate() folds
// in NezhaMotorPort::position() (via DiffDrive::DifferentialDrive::
// Output), which EncoderGlitchArmor (encoder_glitch_armor.h, sprint 006
// ticket 005) already keeps CONTINUOUS across a detected raw-counts
// discontinuity: a two-strike kAcceptAsRebaseline decision re-anchors
// encOffset_ so position holds at its last value instead of integrating
// the jump as motion (nezha_port.cpp's collect()). The guarantee is
// inherited from that fix, not re-implemented here.
//
// selectPoseSource() below is the small, pure, host-testable stand-in for
// engineGoToW()'s own selection rule (shims.cpp) -- OtosPort::connected()
// itself cannot be exercised host-side (otos_port.h includes pxt.h
// unconditionally), so this function carries the RULE (a plain ternary)
// in a form a host test can call directly with two fakes, while
// shims.cpp's engineGoToW() calls this exact function rather than
// re-stating the ternary inline, so the two can never drift apart.
//
// Host-portable: motion_engine.h only (itself <cstdint>/<cmath> plus
// diffdrive.h, no pxt.h/CODAL anywhere) -- covered by
// tests/host/test_cxx11_syntax_gate.py via a dedicated syntax-check
// translation unit (tests/host/encoder_pose_source_syntax_check.cpp --
// this header has no natural .cpp of its own), same convention as
// encoder_glitch_armor.h/heading_wrap.h.
#pragma once

#include "../motion/motion_engine.h"

namespace diffDrive {

class EncoderPoseSource : public PoseSource {
 public:
  // Binds to the caller's x/y/heading fields for this instance's entire
  // lifetime -- see this file's header comment on why those fields (and
  // this object itself) must outlive every use of it.
  EncoderPoseSource(const float& x, const float& y, const float& heading)
      : x_(x), y_(y), heading_(heading) {}

  float x() const override { return x_; }  // [mm] world frame, verbatim
  float y() const override { return y_; }  // [mm] world frame, verbatim

  // [rad] world frame, CCW+, UNWRAPPED -- returned exactly as held by the
  // bound field, no normalization applied. See this file's header
  // comment ("HEADING WRAP CONVENTION").
  float heading() const override { return heading_; }

 private:
  const float& x_;
  const float& y_;
  const float& heading_;
};

// The one-line selection rule engineGoToW() (shims.cpp) applies: `primary`
// (OtosPort in production) when `primaryConnected`, `fallback`
// (EncoderPoseSource in production) otherwise. Extracted here, rather than
// left inline at the one call site, purely so a host test can exercise the
// RULE directly against two fakes -- OtosPort::connected() itself has no
// host-testable seam (otos_port.h includes pxt.h unconditionally).
inline PoseSource& selectPoseSource(bool primaryConnected, PoseSource& primary,
                                    PoseSource& fallback) {
  return primaryConnected ? primary : fallback;
}

}  // namespace diffDrive
