// sensor_config.h -- which heading/position sensors this robot has, and the
// boot-time probe that decides. Host-portable: the probe reads only the
// GyroSource / OpticalPositionSource interfaces.
#pragma once

#include "gyro_source.h"

namespace diffDrive {

enum class SensorConfig : uint8_t {
  kOptical,       // optical x/y plus its gyro (OTOS)
  kGyroOnly,      // a gyro, no optical position
  kEncodersOnly,  // wheel encoders alone
};

// A present gyro must also answer one read: a part that identifies itself
// but cannot be read degrades to the next configuration down.
inline SensorConfig probeSensorConfig(GyroSource* gyro,
                                      const OpticalPositionSource* optical) {
  const bool gyroUsable = gyro != nullptr && gyro->present() &&
                          gyro->readGyro() != GyroReadStatus::kFailed;
  if (gyroUsable && optical != nullptr && optical->opticalPresent()) {
    return SensorConfig::kOptical;
  }
  return gyroUsable ? SensorConfig::kGyroOnly : SensorConfig::kEncodersOnly;
}

inline const char* sensorConfigName(SensorConfig config) {
  switch (config) {
    case SensorConfig::kOptical: return "optical";
    case SensorConfig::kGyroOnly: return "gyro";
    case SensorConfig::kEncodersOnly: return "enc";
  }
  return "enc";
}

}  // namespace diffDrive
