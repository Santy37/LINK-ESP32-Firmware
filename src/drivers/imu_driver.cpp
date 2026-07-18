/* IMU Driver — Adafruit BNO055 (9-DOF absolute orientation)
 
 Provides heading / pitch / roll via the on-chip sensor fusion.
 Falls back to DEGRADED if calibration is poor.
 */

#include "config.h"
#if HAS_IMU

#include "imu_driver.h"
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>

static Adafruit_BNO055 bno(55, 0x28, &Wire);  // I²C address 0x28
static bool _initialised = false;
static bool _calibrationHealthy = false;
static uint32_t _calibrationBadSince = 0;

static float normalizeHeading(float degrees) {
  while (degrees < 0.0f)   degrees += 360.0f;
  while (degrees >= 360.0f) degrees -= 360.0f;
  return degrees;
}

bool imu_init() {
  // Wire is initialised once in main setup(); don't re-begin here.

  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("[IMU] BNO055 not detected — FAIL");
    _initialised = false;
    return false;
  }

  bno.setExtCrystalUse(true);
  _initialised = true;
  _calibrationHealthy = false;
  _calibrationBadSince = 0;
  Serial.println("[IMU] BNO055 initialised OK");
  return true;
}

bool imu_selfTest() {
  if (!_initialised) return false;

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // Waypoint bearing depends on fused system + magnetometer calibration.
  // Gyro/accelerometer scores are still logged but do not independently
  // reject an otherwise stable heading.
  bool ok = (sys >= cfg::IMU_MIN_SYS_CAL && mag >= cfg::IMU_MIN_MAG_CAL);
  Serial.printf("[IMU] self-test cal sys=%d gyro=%d accel=%d mag=%d → %s\n",
                sys, gyro, accel, mag, ok ? "PASS" : "DEGRADED");
  return ok;
}

ImuData imu_read() {
  ImuData d{};

  if (!_initialised) {
    d.state = ModuleState::FAIL;
    return d;
  }

  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

  // BNO055 Euler register order is heading, roll, pitch (H/R/P), exposed
  // by Adafruit as orientation.x/y/z respectively.
  // Raw heading is magnetic north; apply deployment declination and the
  // measured IMU-to-LiDAR optical-axis offset before waypoint projection.
  d.heading = normalizeHeading(event.orientation.x +
                               cfg::IMU_MAG_DECLINATION_DEG +
                               cfg::IMU_HEADING_OFFSET_DEG);
  d.roll    = event.orientation.y;
  d.pitch   = event.orientation.z + cfg::IMU_PITCH_OFFSET_DEG;

  // Check calibration quality
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  d.sysCal   = sys;
  d.gyroCal  = gyro;
  d.accelCal = accel;
  d.magCal   = mag;

  sensors_event_t gyroEvent{};
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  float gyroRate = sqrtf(gyroEvent.gyro.x * gyroEvent.gyro.x +
                         gyroEvent.gyro.y * gyroEvent.gyro.y +
                         gyroEvent.gyro.z * gyroEvent.gyro.z);
  bool stationary = gyroRate <= cfg::IMU_STILL_GYRO_MAX_RAD_S;

  // Calibration confidence often falls after a calibrated unit is placed on
  // a stationary mount, especially near tripod hardware. Preserve the last
  // known-good state while still, but never bypass initial calibration. Once
  // moving, continuously bad calibration still degrades after the grace time.
  bool calibrationGood = sys >= cfg::IMU_MIN_SYS_CAL &&
                         mag >= cfg::IMU_MIN_MAG_CAL;
  uint32_t now = millis();
  if (calibrationGood) {
    _calibrationHealthy = true;
    _calibrationBadSince = 0;
  } else if (_calibrationHealthy && stationary) {
    _calibrationBadSince = 0;
  } else if (_calibrationHealthy) {
    if (_calibrationBadSince == 0) {
      _calibrationBadSince = now;
    } else if (now - _calibrationBadSince >= cfg::IMU_CAL_DEGRADE_GRACE_MS) {
      _calibrationHealthy = false;
    }
  }

  d.state = _calibrationHealthy ? ModuleState::OK : ModuleState::DEGRADED;

  return d;
}

#endif // HAS_IMU
