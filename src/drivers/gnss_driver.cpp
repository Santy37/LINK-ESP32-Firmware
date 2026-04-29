/* GNSS Driver — u-blox NEO-M8N / NEO-6M via TinyGPS++
 
 Reads NMEA over UART2, exposes lat/lon/alt/accuracy/fix.
 */

#include "gnss_driver.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "config.h"

static HardwareSerial gpsSerial(2);   // UART2
static TinyGPSPlus    gps;
static bool _initialised = false;

bool gnss_init() {
  // Simple init — matches the working test sketch exactly
  gpsSerial.begin(cfg::GNSS_BAUD, SERIAL_8N1, cfg::GNSS_RX, cfg::GNSS_TX);

  Serial.printf("[GNSS] UART2 started: baud=%ld RX=%d TX=%d\n",
                cfg::GNSS_BAUD, cfg::GNSS_RX, cfg::GNSS_TX);
  Serial.print("[GNSS] Waiting 3s for NMEA data: ");

  // Read raw for 3 seconds — print first 120 chars so we can see what arrives
  int rawCount = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      gps.encode(c);
      if (rawCount < 120) Serial.print(c);
      rawCount++;
    }
  }

  Serial.printf("\n[GNSS] raw chars received: %d  TinyGPS processed: %lu\n",
                rawCount, gps.charsProcessed());

  _initialised = (gps.charsProcessed() > 10);
  Serial.printf("[GNSS] init → %s\n", _initialised ? "OK" : "FAIL (no NMEA)");
  return _initialised;
}

bool gnss_selfTest() {
  if (!_initialised) return false;
  // Consider passing if we've decoded at least one sentence
  bool ok = (gps.sentencesWithFix() > 0 || gps.satellites.value() > 0);
  Serial.printf("[GNSS] self-test sats=%d fix=%s → %s\n",
                (int)gps.satellites.value(),
                gps.location.isValid() ? "valid" : "none",
                ok ? "PASS" : "NO FIX YET");
  return ok;
}

GnssData gnss_read() {
  // Drain available bytes
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  GnssData d{};

  if (!_initialised) {
    d.state = ModuleState::FAIL;
    return d;
  }

  if (gps.location.isValid()) {
    d.lat  = gps.location.lat();
    d.lon  = gps.location.lng();
    d.altM = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0;
    d.sats = (uint8_t)gps.satellites.value();

    // Rough accuracy from HDOP (metres ≈ HDOP × 5)
    d.accM = gps.hdop.isValid() ? (float)(gps.hdop.hdop() * 5.0) : 99.0f;

    // Determine fix quality
    if (d.sats >= 6)      d.fix = GnssFix::FIX_3D;
    else if (d.sats >= 3) d.fix = GnssFix::FIX_2D;
    else                   d.fix = GnssFix::NONE;

    d.state = (d.fix >= GnssFix::FIX_2D) ? ModuleState::OK : ModuleState::DEGRADED;
  } else {
    d.fix   = GnssFix::NONE;
    d.state = ModuleState::DEGRADED;
  }

  return d;
}

unsigned long gps_charsProcessed() {
  return gps.charsProcessed();
}
