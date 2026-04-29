/* Barometer Driver — BMP280 over I²C
 
 Provides pressure (hPa), temperature (°C), and barometric altitude.
 */

#include "config.h"
#if HAS_BARO

#include "baro_driver.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>

static Adafruit_BMP280 bmp(&Wire);
static bool _initialised = false;
static bool _calibrated  = false;   // true once GPS has set an absolute MSL reference

// Sea-level reference — starts at ISA standard; gets overwritten by GPS
static float _seaLevelHPa = 1013.25f;

bool baro_init() {
  if (!bmp.begin(0x76)) {       // try 0x76 first, then 0x77
    if (!bmp.begin(0x77)) {
      Serial.println("[BARO] BMP280 not found — FAIL");
      _initialised = false;
      return false;
    }
  }

  // High-resolution oversampling for altitude accuracy
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X16,   // temp
                  Adafruit_BMP280::SAMPLING_X16,   // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);

  _initialised = true;

  // Let the oversampling filter warm up so self-test reads a stable value
  delay(100);
  for (int i = 0; i < 8; ++i) { (void)bmp.readPressure(); delay(20); }

  Serial.println("[BARO] BMP280 initialised OK (awaiting GPS calibration for absolute altitude)");
  return true;
}

bool baro_selfTest() {
  if (!_initialised) return false;
  float p = bmp.readPressure() / 100.0f;
  bool ok = (p > 300.0f && p < 1100.0f);   // sane range
  Serial.printf("[BARO] self-test pressure=%.1f hPa → %s\n", p, ok ? "PASS" : "FAIL");
  return ok;
}

void baro_setSeaLevel(float hPa) {
  _seaLevelHPa = hPa;
  _calibrated  = true;
}

bool baro_isCalibrated() {
  return _calibrated;
}

float baro_currentPressure() {
  if (!_initialised) return 0;
  return bmp.readPressure() / 100.0f;
}

BaroData baro_read() {
  BaroData d{};

  if (!_initialised) {
    d.state = ModuleState::FAIL;
    return d;
  }

  d.pressHPa = bmp.readPressure() / 100.0f;
  d.tempC    = bmp.readTemperature();

  bool pressureOk = (d.pressHPa > 300.0f && d.pressHPa < 1100.0f);
  if (!pressureOk) {
    d.altEstM = 0;
    d.state   = ModuleState::DEGRADED;
  } else {
    /* Always report an altitude using the current sea-level reference.
     Until GPS calibrates us, this is "pressure altitude" (ISA-standard,
     ±300 m worst case).  After GPS calibration, it's true MSL altitude.
     Either way, the baro is independently usable.
     */
    d.altEstM = bmp.readAltitude(_seaLevelHPa);
    d.state   = ModuleState::OK;
  }
  return d;
}

#endif // HAS_BARO
