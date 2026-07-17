/* L.I.N.K. ESP32-S3 Firmware — Pin / Configuration Definitions
 
 All GPIO assignments, timing constants, and tunables live here
 so nothing is scattered across driver files.
 */

#pragma once
#include <cstdint>

namespace cfg {

/* Hardware feature flags
   Set to 0 to disable a module you don't have connected.
   Disabled modules return stub/simulated data so BLE still works.
 */
#define HAS_IMU    1    // BNO055 9-DOF — connected (I²C 0x28)
#define HAS_BARO   1    // BME280 — connected (I²C 0x76/0x77)
#define HAS_LIDAR  1    // PTYS-12X — connected (UART1)
#define HAS_GNSS   1    // NEO-M9N GPS — connected
#define HAS_OLED   0    // SSD1306 128x64 I²C OLED  (legacy — disable when TFT proven)
#define HAS_TFT    1    // ST7735 1.8" 128x160 RGB TFT (SPI)

/* Aiming calibration
   BNO055 heading is magnetic north. Convert it to the true bearing used by
   map coordinates with local magnetic declination (east positive, west
   negative), then add the fixed IMU-to-LiDAR boresight alignment offset.
   The declination below is NOAA WMM-2025 for UCF (28.6012, -81.2005) on
   2026-07-17: -7.0766 degrees (west). Update it if operating elsewhere.
   Keep the boresight offset separate and tune it only for mounting error.
 */
constexpr float IMU_MAG_DECLINATION_DEG = -7.08f;
constexpr float IMU_HEADING_OFFSET_DEG  = 0.0f;
constexpr float IMU_PITCH_OFFSET_DEG    = 0.0f;
constexpr uint8_t IMU_MIN_SYS_CAL       = 1;  // sys=0 has not found magnetic north
constexpr uint8_t IMU_MIN_MAG_CAL       = 2;  // reject weak/unstable compass calibration
constexpr uint32_t IMU_CAL_DEGRADE_GRACE_MS = 5000; // ignore brief calibration dips

/* Ping input quality gates. Stale cached data must never produce a waypoint. */
constexpr uint32_t GNSS_MAX_LOCATION_AGE_MS = 2000;
constexpr uint8_t  PING_GNSS_MIN_SATS       = 6;
constexpr float    PING_GNSS_MAX_ACC_M      = 15.0f;
constexpr uint32_t LIDAR_MAX_SAMPLE_AGE_MS  = 500;

/* GPS-based baro calibration gate.
 Calibration only runs when the GPS fix is confident enough to trust
 its altitude reading.  Until then, baro altitude falls back to raw GPS.
 
 The thresholds here are vibe-tuned: 6 sats + 15m horizontal accuracy
 is the sweet spot where vertical accuracy is usable. Below that, GPS
 altitude is too noisy to calibrate against.
 */
constexpr uint8_t  BARO_CAL_MIN_SATS     = 6;      // need solid satellite count
constexpr float    BARO_CAL_MAX_ACC_M    = 15.0f;  // horizontal accuracy ceiling
constexpr uint8_t  BARO_CAL_SAMPLES      = 5;      // average this many GPS fixes
constexpr uint32_t BARO_RECAL_INTERVAL_MS = 600000UL; // re-run every 10 min (weather drift)

// UART / debug
constexpr long SERIAL_BAUD = 115200;

/* I²C bus (shared: OLED + IMU / Baro)
   NOTE: ESP32-S3-WROOM-2 (N32R16V) does NOT expose GPIO 22-25.
   Use any two free GPIOs — 8 & 9 sit next to each other on the
   DevKitC-1 header and have no special boot-strapping function.
 */
constexpr int I2C_SDA = 8; //THIS IS FOR BAROMETER, OLED, AND IMY
constexpr int I2C_SCL = 9;
constexpr uint32_t I2C_FREQ = 400000;  // 400 kHz Fast-mode

// GNSS (UART2) — GT-U7 (u-blox compatible, 9600 baud NMEA)
constexpr int GNSS_RX = 16;   // ESP RX ← GPS TX
constexpr int GNSS_TX = 17;   // ESP TX → GPS RX
constexpr long GNSS_BAUD = 38400;

/* LiDAR (UART1) — PTYS-12X (Benewake-style 8-byte framed protocol)
   On ESP32-S3-WROOM-2 (N32R16V) DevKitC, GPIO 48 is the onboard WS2812
   RGB LED — it fights any UART driver and pulls TX to ~1.8 V. Avoid it.
   GPIO 21 is clean (no strapping, no USB, no PSRAM, no LED).
   Baud is autodetected at boot from {115200, 9600, 38400, 57600}.
   LIDAR_BAUD below is just the fallback if autodetect fails.
 */
constexpr int LIDAR_RX = 47;  // ESP RX ← LiDAR TX (PTYS pin 3, TTL_TXD)
constexpr int LIDAR_TX = 21;  // ESP TX → LiDAR RX (PTYS pin 2, TTL_RXD)
constexpr long LIDAR_BAUD = 115200;  // fallback only — see lidar_driver.cpp

/* HUD control buttons (3× momentary tactile, active-LOW with INPUT_PULLUP)
   PREV → cycle HUD page backwards.
   NEXT → cycle HUD page forwards.
   PING → hold for PING_HOLD_MS to drop a waypoint pin.
   Pins 4 / 5 / 7 are reused from the previous rotary-encoder wiring.
 */
#define HAS_BUTTONS 1
constexpr int BTN_PREV_PIN = 4;        // page --
constexpr int BTN_NEXT_PIN = 5;        // page ++
constexpr int BTN_PING_PIN = 7;        // hold to ping
constexpr unsigned long PING_HOLD_MS = 1250;  // 1.25-second hold to confirm ping

// Ping button (legacy — kept as fallback if encoder absent)
constexpr int PING_BTN_PIN = 0;        // active-LOW (built-in BOOT btn)
constexpr unsigned long DEBOUNCE_MS = 250;

/* Active buzzer (SMT-0440-T-R via MMBT2222A driver, or a bare 3-pin
   "low-level trigger" module). Active buzzers have their own oscillator,
   so we just toggle the pin — no PWM / ledc / tone() needed.
   The module used here is LOW-triggered: pin LOW → beep, HIGH → silent.
 */
#define HAS_BUZZER 1
constexpr int  BUZZER_PIN         = 13;   // BUZ_CTRL → transistor base
constexpr bool BUZZER_ACTIVE_LOW  = true; // module beeps when pin is LOW
constexpr unsigned long BUZZER_BEEP_MS      = 80;   // one short chirp
constexpr unsigned long BUZZER_BEEP_GAP_MS  = 60;   // gap between chirps
constexpr unsigned long BUZZER_FAIL_MS      = 300;  // one long error tone

/* Battery ADC
   GPIO 33-37 are used by Octal PSRAM on N32R16V.
   GPIO 4 = ADC1_CH3 — safe and accessible on the header.
 */
constexpr int BATT_ADC_PIN = 6;        // voltage divider mid-point
constexpr float BATT_VDIV_RATIO = 2.0; // R1 = R2 → ×2
constexpr float BATT_FULL_V = 4.2;
constexpr float BATT_EMPTY_V = 3.3;

// HUD display — SSD1306 I²C OLED 128×64 (legacy)
constexpr int OLED_WIDTH  = 128;
constexpr int OLED_HEIGHT = 64;
constexpr int OLED_ADDR   = 0x3C;  // common I²C address for SSD1306
// Uses the same I²C bus as IMU/Baro (SDA/SCL above)

/* HUD display — ST7735 1.8" 128×160 RGB TFT (SPI)
   Pins below MUST match the TFT_eSPI build_flags in platformio.ini.
   BL (backlight) is tied to 3.3 V on the module → no pin needed.
   Uses HSPI (VSPI is on internal flash). Driven by TFT_eSPI library.

   Rotation: 0/2 = portrait 128w × 160h; 1/3 = landscape 160w × 128h.
 */
constexpr int  TFT_PIN_SCLK  = 12;   // SPI clock
constexpr int  TFT_PIN_MOSI  = 11;   // SPI MOSI (data)
constexpr int  TFT_PIN_MISO  = -1;   // not used (display is write-only)
constexpr int  TFT_PIN_CS    = 10;   // chip-select
constexpr int  TFT_PIN_DC    = 14;   // data / command
constexpr int  TFT_PIN_RST   = 15;   // reset
constexpr int  TFT_PIN_BL    = -1;   // backlight tied to 3.3 V
constexpr int  TFT_W         = 128;
constexpr int  TFT_H         = 160;
constexpr int  TFT_ROTATION  = 3;    // 3=landscape rotated 180 degrees.
constexpr bool TFT_MIRROR_HORIZONTAL = true;  // Mirror left-to-right for HUD optics.

// BLE
constexpr const char* BLE_DEVICE_NAME = "LINK-HUD";

// 128-bit UUIDs — must match hud-app/src/lib/ble.ts exactly
constexpr const char* BLE_SERVICE_UUID        = "4c494e4b-4855-4400-b000-000000000000";
constexpr const char* BLE_TELEMETRY_CHAR_UUID = "4c494e4b-4855-4400-b000-000000000001";
constexpr const char* BLE_PIN_CHAR_UUID       = "4c494e4b-4855-4400-b000-000000000002";
constexpr const char* BLE_ACK_CHAR_UUID       = "4c494e4b-4855-4400-b000-000000000003";
constexpr const char* BLE_CAL_CHAR_UUID       = "4c494e4b-4855-4400-b000-000000000004";

// Timing
// Telemetry @ 5 Hz is plenty smooth on the HUD without saturating BLE.
// Sensor @ 20 Hz so we never miss a button press or rotation tick.
// Retry every 10s so unsent pins eventually deliver without spamming.
constexpr unsigned long TELEMETRY_INTERVAL_MS = 200;   // 5 Hz streaming
constexpr unsigned long QUEUE_RETRY_INTERVAL_MS = 10000; // retry unsent pins every 10s
constexpr unsigned long SENSOR_READ_INTERVAL_MS = 50;   // 20 Hz sensor polling
constexpr int MAX_LOCAL_QUEUE = 32;                      // max queued pins on ESP32

// Waypoint computation
constexpr float EARTH_RADIUS_M = 6371000.0f;

}  // namespace cfg
