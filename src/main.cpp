/* L.I.N.K. ESP32-S3 Firmware — main.cpp
 *
 * Boot sequence:
 *   1. Init serial + peripherals (I²C, UART)
 *   2. Init each sensor driver
 *   3. Run self-tests — degrade if any fail, warn via Serial (+ HUD later)
 *   4. Init BLE GATT server + start advertising
 *   5. Enter main loop:
 *      a. Poll sensors (20 Hz)
 *      b. Stream telemetry over BLE (5 Hz)
 *      c. On ping button → snapshot → compute waypoint → queue → BLE notify
 *      d. Retry un-ACK'd pins periodically
 *
 * Matches the phone-side pipeline in:
 *   hud-app/src/lib/ble.ts
 *   hud-app/src/lib/payloadTypes.ts
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "sensor_types.h"

// Drivers (conditionally included)
#if HAS_IMU
  #include "drivers/imu_driver.h"
#endif
#if HAS_GNSS
  #include "drivers/gnss_driver.h"
#endif
#if HAS_BARO
  #include "drivers/baro_driver.h"
#endif
#if HAS_LIDAR
  #include "drivers/lidar_driver.h"
#endif

// OLED display
#if HAS_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  static Adafruit_SSD1306 oled(cfg::OLED_WIDTH, cfg::OLED_HEIGHT, &Wire, -1);
  static bool oledOk = false;
#endif

// Rotary encoder
#if HAS_ENCODER
  #include "drivers/encoder_driver.h"
#endif

// BLE + logic
#include "ble_server.h"
#include "telemetry_json.h"
#include "waypoint.h"
#include "pin_queue.h"
// Forward declarations
void handlePing();
void retryPendingPins();
// Timing state
static unsigned long lastSensorRead   = 0;
static unsigned long lastTelemetrySend = 0;
static unsigned long lastQueueRetry    = 0;
static unsigned long lastPingMs        = 0;   // debounce
static bool baroCalibratedFromGps     = false;

// HUD page state
static const int HUD_PAGE_COUNT = 5;
static int       hudPage        = 0;   // 0..4
static bool      pingHoldActive = false;
static unsigned long pingHoldStart = 0;

// Latest sensor snapshot
static TelemetrySnapshot snap;

// Stub sensor data for modules not connected
static ImuData   imu_stub()   { ImuData d{}; d.heading=0; d.pitch=0; d.roll=0; d.state=ModuleState::FAIL; return d; }
static BaroData  baro_stub()  { BaroData d{}; d.pressHPa=1013.25; d.tempC=25; d.altEstM=0; d.state=ModuleState::FAIL; return d; }
static LidarData lidar_stub() { LidarData d{}; d.rangeM=0; d.quality=0; d.valid=false; d.state=ModuleState::FAIL; return d; }
static GnssData  gnss_stub()  { GnssData d{}; d.lat=0; d.lon=0; d.altM=0; d.accM=99; d.fix=GnssFix::NONE; d.sats=0; d.state=ModuleState::FAIL; return d; }

// Battery
static uint8_t readBatteryPercent() {
  // TODO: wire voltage divider to GPIO 4 for real battery reading
  return 100;
}

// UUID generator (pseudo — no true RNG, but good enough)
static String generateUUID() {
  char buf[37];
  const char* hex = "0123456789abcdef";
  for (int i = 0; i < 36; i++) {
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      buf[i] = '-';
    } else {
      buf[i] = hex[esp_random() % 16];
    }
  }
  buf[36] = '\0';
  return String(buf);
}

// SETUP — boot, init, self-test, BLE advertise

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);

  /* Native USB CDC on ESP32-S3 needs time to enumerate after reset.
   * Wait up to 3 seconds for a host to open the port, then continue
   * headless so the firmware still boots without a PC attached.
   */
  unsigned long usbWait = millis();
  while (!Serial && (millis() - usbWait < 3000)) { delay(10); }
  delay(300);                     // extra settle time

  Serial.println();
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║   L.I.N.K. Binocular HUD — Firmware     ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println();

  // Ping button (fallback)
  pinMode(cfg::PING_BTN_PIN, INPUT_PULLUP);

  // Rotary encoder
#if HAS_ENCODER
  encoder_init();
#endif

  // Turn off onboard RGB LED (GPIO 38)
  neopixelWrite(38, 0, 0, 0);

  // Battery ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // I²C bus
  Wire.begin(cfg::I2C_SDA, cfg::I2C_SCL);
  Wire.setClock(cfg::I2C_FREQ);
  Serial.printf("[I2C]  SDA=%d  SCL=%d  %lukHz\n",
                cfg::I2C_SDA, cfg::I2C_SCL, cfg::I2C_FREQ / 1000);

  // OLED display
#if HAS_OLED
  if (oled.begin(SSD1306_SWITCHCAPVCC, cfg::OLED_ADDR)) {
    oledOk = true;
    oled.ssd1306_command(SSD1306_SETCONTRAST);
    oled.ssd1306_command(0xFF);
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("L.I.N.K. Booting...");
    oled.display();
    Serial.println("[OLED] SSD1306 initialised OK");
  } else {
    Serial.printf("[OLED] SSD1306 not found at 0x%02X — FAIL\n", cfg::OLED_ADDR);
  }
#endif

  // Sensor init + self-test
  Serial.println("\n── Initialising sensors ──────────────────");

#if HAS_IMU
  bool imuOk   = imu_init();
#else
  bool imuOk   = false;
  Serial.println("[IMU]   disabled (HAS_IMU=0)");
#endif

#if HAS_GNSS
  bool gnssOk  = gnss_init();
#else
  bool gnssOk  = false;
  Serial.println("[GNSS]  disabled (HAS_GNSS=0)");
#endif

#if HAS_BARO
  bool baroOk  = baro_init();
#else
  bool baroOk  = false;
  Serial.println("[BARO]  disabled (HAS_BARO=0)");
#endif

#if HAS_LIDAR
  bool lidarOk = lidar_init();
#else
  bool lidarOk = false;
  Serial.println("[LIDAR] disabled (HAS_LIDAR=0)");
#endif

  Serial.println();
  Serial.println("── Running self-tests ───────────────────");

#if HAS_IMU
  bool imuTest   = imuOk   && imu_selfTest();
#else
  bool imuTest   = false;
#endif

#if HAS_GNSS
  bool gnssTest  = gnssOk  && gnss_selfTest();
#else
  bool gnssTest  = false;
#endif

#if HAS_BARO
  bool baroTest  = baroOk  && baro_selfTest();
#else
  bool baroTest  = false;
#endif

#if HAS_LIDAR
  bool lidarTest = lidarOk && lidar_selfTest();
#else
  bool lidarTest = false;
#endif

  Serial.println();
  Serial.println("── Self-test summary ────────────────────");
  Serial.printf("  IMU   : %s\n", imuTest   ? "PASS" : (imuOk   ? "DEGRADED" : "DISABLED"));
  Serial.printf("  GNSS  : %s\n", gnssTest  ? "PASS" : (gnssOk  ? "DEGRADED" : "DISABLED"));
  Serial.printf("  BARO  : %s\n", baroTest  ? "PASS" : (baroOk  ? "DEGRADED" : "DISABLED"));
  Serial.printf("  LiDAR : %s\n", lidarTest ? "PASS" : (lidarOk ? "DEGRADED" : "DISABLED"));
  Serial.println();

  /* If a module failed init entirely, it will report FAIL in telemetry.
     The phone UI and HUD will show warnings to the user.                */

  // BLE
  ble_init();

  /* Register ACK callback → mark pin as delivered in local queue */
  ble_onAck([](const String& pinId) {
    pinQueue_ack(pinId);
    pinQueue_prune();
  });

  ble_onConnection([](bool connected) {
    if (connected) {
      Serial.printf("[MAIN] Phone connected — %d pending pins in queue\n",
                    pinQueue_pendingCount());
    }
  });

  Serial.println("[MAIN] Setup complete — entering main loop\n");

#if HAS_OLED
  if (oledOk) {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("L.I.N.K. Ready");
    oled.printf("GPS: %s\n", gnssOk ? "OK" : "--");
    oled.println("Waiting for BLE...");
    oled.display();
  }
#endif
}

// LOOP — sensor polling, telemetry streaming, ping handling, retries

void loop() {
  unsigned long now = millis();

  // 1. Poll sensors (20 Hz)
  if (now - lastSensorRead >= cfg::SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;

#if HAS_IMU
    snap.imu   = imu_read();
#else
    snap.imu   = imu_stub();
#endif
#if HAS_GNSS
    snap.gnss  = gnss_read();
#else
    snap.gnss  = gnss_stub();
#endif
#if HAS_BARO
    snap.baro  = baro_read();
#else
    snap.baro  = baro_stub();
#endif
#if HAS_LIDAR
    snap.lidar = lidar_read();
#else
    snap.lidar = lidar_stub();
#endif
    snap.battery = readBatteryPercent();

    // Auto-calibrate baro from GPS altitude when fix is trustworthy.
    // This upgrades baro from "pressure altitude" (± standard atmosphere error)
    // to true MSL altitude.  Baro still works independently if GPS never comes.
    // Re-runs every BARO_RECAL_INTERVAL_MS so baro tracks weather drift.
#if HAS_GNSS && HAS_BARO
    {
      static unsigned long lastBaroCalMs = 0;
      static float   altSum   = 0.0f;
      static uint8_t altCount = 0;
      bool timeToRecal = !baroCalibratedFromGps ||
                         (now - lastBaroCalMs >= cfg::BARO_RECAL_INTERVAL_MS);
      bool gpsGood = snap.gnss.fix >= GnssFix::FIX_3D &&
                     snap.gnss.sats >= cfg::BARO_CAL_MIN_SATS &&
                     snap.gnss.accM > 0 && snap.gnss.accM <= cfg::BARO_CAL_MAX_ACC_M;
      if (timeToRecal && gpsGood) {
        altSum += snap.gnss.altM;
        altCount++;
        if (altCount >= cfg::BARO_CAL_SAMPLES) {
          float avgAlt = altSum / altCount;
          float p = baro_currentPressure();
          if (p > 300.0f) {
            float slp = p / powf(1.0f - avgAlt / 44330.0f, 5.255f);
            baro_setSeaLevel(slp);
            baroCalibratedFromGps = true;
            lastBaroCalMs = now;
            Serial.printf("[BARO] Calibrated from GPS avg: alt=%.1fm (n=%u) pressure=%.1f → SLP=%.2f hPa\n",
                          avgAlt, altCount, p, slp);
          }
          altSum = 0.0f;
          altCount = 0;
        }
      }
    }
#endif
  }

  // 2. Stream telemetry over BLE (5 Hz)
  if (now - lastTelemetrySend >= cfg::TELEMETRY_INTERVAL_MS) {
    lastTelemetrySend = now;

    if (ble_isConnected()) {
      String json = telemetry_toJson(snap);
      ble_sendTelemetry(json);
    }
  }

  // 3. Rotary encoder: page change + long-press ping
#if HAS_ENCODER
  {
    // Rotation → cycle HUD pages (clamp to ±1 so one click = one page)
    int rot = encoder_getRotation();
    if (rot != 0) {
      rot = (rot > 0) ? 1 : -1;
      hudPage = (hudPage + rot + HUD_PAGE_COUNT) % HUD_PAGE_COUNT;
      Serial.printf("[ENC]  Page → %d\n", hudPage);
    }

    // Long-press → trigger ping
    bool longPress = encoder_poll();

    // Show hold feedback on OLED while button is physically held
    bool btnHeld = (digitalRead(cfg::ENC_SW_PIN) == LOW);
    if (btnHeld && !pingHoldActive) {
      pingHoldActive = true;
      pingHoldStart  = now;
    }
    if (!btnHeld) {
      pingHoldActive = false;
    }

    if (longPress && (now - lastPingMs) > cfg::DEBOUNCE_MS) {
      lastPingMs = now;
      handlePing();
    }
  }
#else
  /* Fallback: BOOT button short-press */
  if (digitalRead(cfg::PING_BTN_PIN) == LOW &&
      (now - lastPingMs) > cfg::DEBOUNCE_MS) {
    lastPingMs = now;
    handlePing();
  }
#endif

  // 4. Retry un-ACK'd pins
  if (now - lastQueueRetry >= cfg::QUEUE_RETRY_INTERVAL_MS) {
    lastQueueRetry = now;
    retryPendingPins();
  }

  // 5. Debug log (~5s) — all systems, not just GPS
  static unsigned long lastDebug = 0;
  if (now - lastDebug >= 5000) {
    lastDebug = now;
    Serial.println("┌─── System Status ───────────────────────");
    Serial.printf("│ BLE   : %s\n", ble_isConnected() ? "CONNECTED" : "advertising");
    Serial.printf("│ Batt  : %d%%\n", snap.battery);
#if HAS_IMU
    Serial.printf("│ IMU   : hdg=%.1f° pit=%.1f° rol=%.1f° [%s]\n",
                  snap.imu.heading, snap.imu.pitch, snap.imu.roll,
                  snap.imu.state == ModuleState::OK ? "OK" :
                  snap.imu.state == ModuleState::DEGRADED ? "DEGRADED" : "FAIL");
#else
    Serial.println("│ IMU   : disabled");
#endif
#if HAS_GNSS
    Serial.printf("│ GNSS  : sats=%d fix=%s lat=%.6f lon=%.6f chars=%lu [%s]\n",
                  snap.gnss.sats,
                  snap.gnss.fix >= GnssFix::FIX_2D ? "YES" : "no",
                  snap.gnss.lat, snap.gnss.lon,
                  gps_charsProcessed(),
                  snap.gnss.state == ModuleState::OK ? "OK" :
                  snap.gnss.state == ModuleState::DEGRADED ? "DEGRADED" : "FAIL");
#else
    Serial.println("│ GNSS  : disabled");
#endif
#if HAS_BARO
    Serial.printf("│ BARO  : %.1f hPa  %.1f°C  alt=%.1fm [%s]\n",
                  snap.baro.pressHPa, snap.baro.tempC, snap.baro.altEstM,
                  snap.baro.state == ModuleState::OK ? "OK" :
                  snap.baro.state == ModuleState::DEGRADED ? "DEGRADED" : "FAIL");
#else
    Serial.println("│ BARO  : disabled");
#endif
#if HAS_LIDAR
    Serial.printf("│ LiDAR : range=%.2fm qual=%d valid=%d [%s]\n",
                  snap.lidar.rangeM, snap.lidar.quality, snap.lidar.valid,
                  snap.lidar.state == ModuleState::OK ? "OK" :
                  snap.lidar.state == ModuleState::DEGRADED ? "DEGRADED" : "FAIL");
#else
    Serial.println("│ LiDAR : disabled");
#endif
    Serial.printf("│ Queue : %d pending pins\n", pinQueue_pendingCount());
    Serial.println("└──────────────────────────────────────────");
  }
  // 5. Update OLED display (~2 Hz) — multi-page HUD
#if HAS_OLED
  static unsigned long lastOled = 0;
  if (oledOk && now - lastOled >= 500) {
    lastOled = now;
    oled.clearDisplay();
    oled.setCursor(0, 0);

    // Ping-hold overlay (takes over entire screen)
#if HAS_ENCODER
    if (pingHoldActive) {
      unsigned long held = now - pingHoldStart;
      int pct = constrain((int)(held * 100 / cfg::ENC_LONG_PRESS_MS), 0, 100);
      oled.setTextSize(1);
      oled.println();
      oled.println("  CONFIRMING PING");
      oled.println("  Please hold...");
      oled.println();
      // Progress bar (100 px wide, centred)
      int barX = 14, barY = 36, barW = 100, barH = 10;
      oled.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
      oled.fillRect(barX + 2, barY + 2, (barW - 4) * pct / 100, barH - 4, SSD1306_WHITE);
      oled.setCursor(barX, barY + barH + 4);
      oled.printf("         %d%%", pct);
      oled.display();
      return;  // skip normal page rendering this frame
    }
#endif

    // Page header
    oled.setTextSize(1);
    oled.printf("[%d/%d] ", hudPage + 1, HUD_PAGE_COUNT);

    switch (hudPage) {

    // Page 0: Overview
    case 0:
      oled.println("OVERVIEW");
      oled.printf("BLE:%s GPS:%s\n",
        ble_isConnected() ? "ON " : "---",
        snap.gnss.fix >= GnssFix::FIX_2D ? "FIX" : "---");
      if (snap.gnss.fix >= GnssFix::FIX_2D)
        oled.printf("%.5f,%.5f\n", snap.gnss.lat, snap.gnss.lon);
      else
        oled.printf("Sats: %d\n", snap.gnss.sats);
      oled.printf("Hdg: %.0f  Pit:%.0f\n", snap.imu.heading, snap.imu.pitch);
      oled.printf("Alt: %.0fm  %.0fhPa\n", snap.baro.altEstM, snap.baro.pressHPa);
      oled.printf("Rng: %.1fm  Q:%d\n",
        snap.lidar.valid ? snap.lidar.rangeM : 0.0f,
        pinQueue_pendingCount());
      oled.printf("Bat: %d%%\n", snap.battery);
      break;

    // Page 1: IMU detail
    case 1:
      oled.println("IMU");
      oled.println();
      oled.printf("Heading: %.1f deg\n", snap.imu.heading);
      oled.printf("Pitch:   %.1f deg\n", snap.imu.pitch);
      oled.printf("Roll:    %.1f deg\n", snap.imu.roll);
      oled.println();
      oled.printf("State: %s\n",
        snap.imu.state == ModuleState::OK ? "OK" :
        snap.imu.state == ModuleState::DEGRADED ? "DEGRADED" : "FAIL");
      break;

    // Page 2: GNSS detail
    case 2:
      oled.println("GNSS");
      oled.println();
      oled.printf("Lat: %.6f\n", snap.gnss.lat);
      oled.printf("Lon: %.6f\n", snap.gnss.lon);
      oled.printf("Alt: %.1f m\n", snap.gnss.altM);
      oled.printf("Acc: %.1f m  Sat:%d\n", snap.gnss.accM, snap.gnss.sats);
      oled.printf("Fix: %s\n",
        snap.gnss.fix == GnssFix::FIX_3D ? "3D" :
        snap.gnss.fix == GnssFix::FIX_2D ? "2D" : "NONE");
      break;

    // Page 3: Baro detail
    case 3:
      oled.println("BAROMETER");
      oled.println();
      oled.printf("Press: %.1f hPa\n", snap.baro.pressHPa);
      oled.printf("Temp:  %.1f C\n", snap.baro.tempC);
      oled.printf("Alt:   %.1f m\n", snap.baro.altEstM);
      oled.println();
      oled.printf("State: %s\n",
        snap.baro.state == ModuleState::OK ? "OK" :
        snap.baro.state == ModuleState::DEGRADED ? "DEGRADED" : "FAIL");
      break;

    // Page 4: LiDAR / Range
    case 4:
      oled.println("LiDAR");
      oled.println();
      oled.printf("Range:   %.2f m\n", snap.lidar.rangeM);
      oled.printf("Quality: %d\n", snap.lidar.quality);
      oled.printf("Valid:   %s\n", snap.lidar.valid ? "YES" : "NO");
      oled.println();
      oled.printf("State: %s\n",
        snap.lidar.state == ModuleState::OK ? "OK" :
        snap.lidar.state == ModuleState::DEGRADED ? "DEGRADED" : "FAIL");
      break;
    }

    oled.display();
  }
#endif
}

// handlePing — user pressed the ping button

void handlePing() {
  Serial.println("\n═══ PING BUTTON PRESSED ══════════════════");

  // Validate measurements
  if (snap.gnss.state == ModuleState::FAIL) {
    Serial.println("[PING] GNSS not available — cannot compute waypoint");
#if HAS_OLED
    if (oledOk) { oled.clearDisplay(); oled.setCursor(0,0); oled.println("PING FAIL"); oled.println("No GPS fix!"); oled.display(); }
#endif
    return;
  }

  /* LiDAR: if not connected, use a default range for testing */
#if HAS_LIDAR
  if (!snap.lidar.valid) {
    Serial.println("[PING] LiDAR reading invalid — cannot compute range");
    return;
  }
  float pingRange = snap.lidar.rangeM;
#else
  // No LiDAR — use a fixed test range of 100m for demo purposes
  float pingRange = 100.0f;
  Serial.println("[PING] LiDAR disabled — using test range of 100m");
#endif

  /* Heading: use IMU if available, otherwise 0° (north) */
#if HAS_IMU
  float pingBearing = snap.imu.heading;
#else
  float pingBearing = 0.0f;
  Serial.println("[PING] IMU disabled — using 0° (north)");
#endif

  // Compute target waypoint
  LatLon target = destinationPoint(
    snap.gnss.lat, snap.gnss.lon,
    pingBearing,
    pingRange
  );

  Serial.printf("[PING] Observer: (%.7f, %.7f)\n", snap.gnss.lat, snap.gnss.lon);
  Serial.printf("[PING] Bearing: %.1f°  Range: %.2f m\n",
                pingBearing, pingRange);
  Serial.printf("[PING] Target:  (%.7f, %.7f)\n", target.lat, target.lon);

  // Generate UUID and serialise payload
  String id   = generateUUID();
  String json = pinPayload_toJson(id, snap, target.lat, target.lon, "Waypoint");

  // Enqueue locally
  pinQueue_enqueue(id, json);

  // Send immediately if phone is connected
  if (ble_isConnected()) {
    ble_sendPinPayload(json);
    Serial.println("[PING] Pin sent over BLE");
  } else {
    Serial.println("[PING] Phone not connected — queued for later");
  }

  Serial.printf("[PING] Queue depth: %d\n\n", pinQueue_pendingCount());
}

// retryPendingPins — resend un-ACK'd pins when phone is connected

void retryPendingPins() {
  if (!ble_isConnected()) return;

  int pending = pinQueue_pendingCount();
  if (pending == 0) return;

  Serial.printf("[RETRY] %d un-ACK'd pins — resending…\n", pending);

  for (int i = 0; i < pending; i++) {
    QueuedPin* p = pinQueue_getPending(i);
    if (!p) break;

    p->attempts++;
    p->lastSentMs = millis();
    ble_sendPinPayload(p->jsonPayload);

    Serial.printf("[RETRY]   pin %s  (attempt #%d)\n", p->id.c_str(), p->attempts);

    delay(50);  // small gap between BLE notifications
  }
}
