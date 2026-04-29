/* IMU Driver — Adafruit BNO055 (9-DOF absolute orientation)
 
 Provides heading / pitch / roll via the on-chip sensor fusion.
 Falls back to DEGRADED if calibration is poor.
 */

#include "config.h"
#if HAS_IMU

#include "imu_driver.h"
#include <Adafruit_BNO055.h>
#include <Wire.h>

static Adafruit_BNO055 bno(55, 0x28, &Wire);  // I²C address 0x28
static bool _initialised = false;

bool imu_init() {
  Wire.begin(cfg::I2C_SDA, cfg::I2C_SCL);
  Wire.setClock(cfg::I2C_FREQ);

  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("[IMU] BNO055 not detected — FAIL");
    _initialised = false;
    return false;
  }

  bno.setExtCrystalUse(true);
  _initialised = true;
  Serial.println("[IMU] BNO055 initialised OK");
  return true;
}

bool imu_selfTest() {
  if (!_initialised) return false;

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // All calibration values should be ≥ 1 for a basic pass
  bool ok = (sys >= 1 && gyro >= 1 && accel >= 1 && mag >= 1);
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

  d.heading = event.orientation.x;   // 0-360
  d.pitch   = event.orientation.y;   // -90 … +90
  d.roll    = event.orientation.z;   // -180 … +180

  // Check calibration quality
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  d.state = (sys >= 2) ? ModuleState::OK : ModuleState::DEGRADED;

  return d;
}

#endif // HAS_IMU
