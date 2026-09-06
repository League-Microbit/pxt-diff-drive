#ifndef DIFFDRIVE_COMMS_CONFIG_FIELDS_H
#define DIFFDRIVE_COMMS_CONFIG_FIELDS_H

#include <cstring>

// The wire's config surface, as data: one row per name a host can reach
// with `SET <name> <value>` / `GET <name>`. This table is the SINGLE
// source of that surface. Three consumers read from it, and none of
// them keeps a second list of its own:
//
//   1. wire_adapter.cpp -- findConfigField() backs onGet()/onSet(), and
//      kConfigFields[] backs the bare-GET dump's field enumeration.
//   2. shims.cpp -- setKernelValue()/getConfigValue() carry the GET/SET
//      BEHAVIOUR for these same ordinals, in their own accessor table.
//      That half cannot live here: it touches Rig, the kernel and the
//      motion engine, so it needs pxt.h, and this header must stay
//      host-portable (it is compiled into every host test that compiles
//      wire_adapter.cpp, and syntax-checked at the target's own C++11).
//      The two halves are bound by ORDINAL, and that binding is
//      enforced, not trusted: tests/host/test_config_surface_single_
//      source.py fails if either side names an ordinal the other does
//      not.
//   3. tools/gen_config_field_enum.py -- generates blocks/motion.ts's
//      `ConfigField` enum from these rows, because PXT compiles a fixed
//      TypeScript file set and cannot read a C++ table at build time.
//      The `ConfigField.<Name>: "<label>"` line above every row is that
//      generator's INPUT, not prose: it supplies the TS member name and
//      the `//% block=` dropdown label, and the generator refuses to run
//      if a row is missing one. Re-run it and commit its output whenever
//      a row is added, renamed or removed;
//      tests/tools/test_gen_config_field_enum.py fails if the
//      checked-in enum and a fresh generation disagree.
//
// Ordinals are a stable wire contract: a removed field's number is
// retired, never reused, so a stale bench script gets `err 1` rather
// than silently writing something else. Declaration order below is the
// order a bare `GET` dumps the surface in.

namespace diffDrive {

struct ConfigFieldDescriptor {
  const char* name;    // the wire key, exactly as SET/GET spell it
  int ordinal;         // setKernelValue()/getConfigValue() field number
  const char* unit;    // of the UNSCALED value; the wire carries it
                       // x1000 (shims.cpp's own boundary convention)
};

constexpr ConfigFieldDescriptor kConfigFields[] = {
    // ConfigField.MaxDuty: "max duty %"
    {"max_duty", 0, "%"},
    // ConfigField.FullDutyVelocity: "full-duty wheel speed"
    {"full_duty_velocity", 1, "counts/s"},
    // ConfigField.Kp: "PID kp"
    {"pid_kp", 2, "1"},
    // ConfigField.Ki: "PID ki"
    {"pid_ki", 3, "1/s"},
    // ConfigField.IMax: "PID integral limit"
    {"pid_i_max", 4, "counts/s"},
    // ConfigField.Kaff: "accel feedforward"
    {"accel_kaff", 5, "s"},
    // ConfigField.PidMax: "PID output limit"
    {"pid_max", 6, "counts/s"},
    // ConfigField.TwistHoldGain: "twist hold gain"
    {"twist_hold_gain", 7, "1/s"},
    // v_floor keeps the ordinal the kernel's own speed floor once had,
    // but writes MotionLimits::vFloor; the kernel's vMin stays 0.
    // ConfigField.VFloor: "speed floor mm/s"
    {"v_floor", 8, "mm/s"},
    // ConfigField.PosErrMax: "position error limit"
    {"pos_err_max", 9, "counts"},
    // ConfigField.StallSpeed: "stall speed"
    {"stall_speed", 10, "counts/s"},
    // ConfigField.StallDemand: "stall demand"
    {"stall_demand", 11, "counts/s"},
    // ConfigField.StallWindow: "stall window ms"
    {"stall_window", 12, "ms"},
    // ConfigField.LambdaEnabled: "lambda enabled"
    {"lambda_enabled", 13, "bool"},
    // ConfigField.CrawlPulse: "crawl pulse"
    {"crawl_pulse", 14, "1"},
    // Backed by Rig::defaultCruise_, not by the kernel's Config.
    // ConfigField.DefaultCruise: "default cruise speed"
    {"default_cruise", 15, "mm/s"},
    // Backed by MotionEngine::rotationalSlip(), not by the kernel's
    // Config.
    // ConfigField.RotationalSlip: "rotational slip"
    {"rotational_slip", 16, "1"},
    // A write-triggered ACTION wearing a config field's clothes: a
    // nonzero SET clears the kernel's stall latch and the magnitude is
    // ignored. Its GET is a convenience readback of the live latch.
    // ConfigField.StallClear: "clear stall latch"
    {"stall_clear", 17, "action"},
    // ConfigField.StopDistance: "stop distance mm"
    {"stop_distance", 18, "mm"},
    // ConfigField.Accel: "acceleration mm/s2"
    {"accel", 19, "mm/s^2"},
    // ConfigField.Decel: "deceleration mm/s2"
    {"decel", 20, "mm/s^2"},
    // ConfigField.VMax: "max speed mm/s"
    {"v_max", 21, "mm/s"},
    // 22-27, 29 and 31 (brake_frac, dist_taper, yaw_taper, dist_floor,
    // turn_floor, ramp_ms, plateau_min_s, profile_exit) are RETIRED
    // ordinals: the fields they named no longer exist, and no row here
    // will ever carry those numbers again.
    // ConfigField.Jerk: "jerk"
    {"jerk", 28, "mm/s^3"},
    // ConfigField.OmegaMax: "max turn rate deg/s"
    {"omega_max", 30, "deg/s"},
    // rebase and estop_clear are the same write-triggered-action shape
    // stall_clear established. Both are refused (kBusy) while a motion
    // is live -- zeroing the frame or dropping the latch out from under
    // an active move is never what the caller meant. rebase alone is
    // also refused on GET: it has no stored value and no latch worth
    // reading back, so answering it would mean manufacturing a 0.
    // ConfigField.Rebase: "zero the pose frame"
    {"rebase", 32, "action"},
    // ConfigField.EstopClear: "clear e-stop latch"
    {"estop_clear", 33, "action"},
    // ConfigField.OmegaFloor: "turn rate floor deg/s"
    {"omega_floor", 34, "deg/s"},
    // ConfigField.ArriveDist: "arrive distance mm"
    {"arrive_dist", 35, "mm"},
    // ConfigField.ArriveYaw: "arrive yaw deg"
    {"arrive_yaw", 36, "deg"},
    // ConfigField.Lag: "response lag (s)"
    {"lag", 37, "s"},
    // ConfigField.StraightTrim: "straight trim"
    {"straight_trim", 38, "1"},
    // The deadline the NEXT go-to gets, backed by Rig::goToDeadline --
    // which the block layer's own engineSetGoToDeadline() shim writes
    // too (that shim exists because every `//%` shim stays at <=4
    // params, not because the value is private). See that field's
    // comment in shims.cpp for what a wire caller can and cannot rely
    // on: a block-issued go-to overwrites it.
    // ConfigField.GoToTimeout: "go-to timeout ms"
    {"goto_timeout", 39, "ms"},
};

constexpr int kConfigFieldCount =
    static_cast<int>(sizeof(kConfigFields) / sizeof(kConfigFields[0]));

// Both finders are `static inline` deliberately: the table above has
// internal linkage (constexpr at namespace scope), so an `inline`
// function with external linkage referring to it would be a different
// entity in every translation unit that included this header. `static`
// keeps each copy honest; `inline` keeps an including-but-not-calling
// translation unit from drawing an unused-function warning.

static inline const ConfigFieldDescriptor* findConfigField(const char* name) {
  for (int i = 0; i < kConfigFieldCount; ++i) {
    if (std::strcmp(name, kConfigFields[i].name) == 0) return &kConfigFields[i];
  }
  return nullptr;
}

static inline const ConfigFieldDescriptor* findConfigFieldByOrdinal(int ordinal) {
  for (int i = 0; i < kConfigFieldCount; ++i) {
    if (kConfigFields[i].ordinal == ordinal) return &kConfigFields[i];
  }
  return nullptr;
}

}  // namespace diffDrive

#endif  // DIFFDRIVE_COMMS_CONFIG_FIELDS_H
