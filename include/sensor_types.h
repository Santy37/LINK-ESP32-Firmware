// Sensor data structures shared across all firmware modules

#pragma once
#include <cstdint>

// Per-module health status
enum class ModuleState : uint8_t {
  OK       = 0,
  DEGRADED = 1,
  FAIL     = 2,
};

const char* moduleStateStr(ModuleState s);

// IMU reading
struct ImuData {
  float heading;   // corrected bearing, degrees clockwise from north (0-360)
  float pitch;     // degrees (-90 … +90)
  float roll;      // degrees (-180 … +180)
  uint8_t sysCal;  // BNO055 calibration levels: 0=uncalibrated, 3=fully calibrated
  uint8_t gyroCal;
  uint8_t accelCal;
  uint8_t magCal;
  ModuleState state = ModuleState::FAIL;
};

// GNSS reading
enum class GnssFix : uint8_t { NONE, FIX_2D, FIX_3D, DGPS };

struct GnssData {
  double lat;
  double lon;
  float  altM;
  float  accM;     // horizontal accuracy estimate (HDOP-derived)
  GnssFix fix     = GnssFix::NONE;
  uint8_t sats    = 0;
  ModuleState state = ModuleState::FAIL;
};

const char* gnssFixStr(GnssFix f);

// Barometer reading
struct BaroData {
  float pressHPa;
  float tempC;
  float altEstM;
  ModuleState state = ModuleState::FAIL;
};

// LiDAR reading
struct LidarData {
  float   rangeM;
  uint8_t quality;   // 0-255
  bool    valid;
  ModuleState state = ModuleState::FAIL;
};

// Aggregate telemetry snapshot (mirrors payloadTypes.ts)
struct TelemetrySnapshot {
  ImuData   imu;
  GnssData  gnss;
  BaroData  baro;
  LidarData lidar;
  uint8_t   battery;  // 0-100 %
};
