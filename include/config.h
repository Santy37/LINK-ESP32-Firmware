/* L.I.N.K. ESP32-S3 Firmware — Pin / Configuration Definitions
 *
 * All GPIO assignments, timing constants, and tunables live here
 * so nothing is scattered across driver files.
 */

#pragma once
#include <cstdint>

namespace cfg {

/* Hardware feature flags
 *   Set to 0 to disable a module you don't have connected.
 *   Disabled modules return stub/simulated data so BLE still works.
 */
#define HAS_IMU    1    // BNO055 9-DOF — connected (I²C 0x28)
#define HAS_BARO   1    // BMP280 — connected (I²C 0x76/0x77)
#define HAS_LIDAR  0    // TFMini — not connected yet
#define HAS_GNSS   1    // GT-U7 GPS — connected
#define HAS_OLED   1    // SSD1306 128x64 I²C OLED

/* GPS-based baro calibration gate.
 * Calibration only runs when the GPS fix is confident enough to trust
 * its altitude reading.  Until then, baro altitude falls back to raw GPS.
 */
constexpr uint8_t  BARO_CAL_MIN_SATS     = 6;      // need solid satellite count
constexpr float    BARO_CAL_MAX_ACC_M    = 15.0f;  // horizontal accuracy ceiling
constexpr uint8_t  BARO_CAL_SAMPLES      = 5;      // average this many GPS fixes
constexpr uint32_t BARO_RECAL_INTERVAL_MS = 600000UL; // re-run calibration every 10 min

// UART / debug
constexpr long SERIAL_BAUD = 115200;

/* I²C bus (shared: OLED + future IMU / Baro)
 *   NOTE: ESP32-S3-WROOM-2 (N32R16V) does NOT expose GPIO 22-25.
 *   Use any two free GPIOs — 8 & 9 sit next to each other on the
 *   DevKitC-1 header and have no special boot-strapping function.
 */
constexpr int I2C_SDA = 8;
constexpr int I2C_SCL = 9;
constexpr uint32_t I2C_FREQ = 400000;  // 400 kHz Fast-mode

// GNSS (UART2) — GT-U7 (u-blox compatible, 9600 baud NMEA)
constexpr int GNSS_RX = 16;   // ESP RX ← GPS TX
constexpr int GNSS_TX = 17;   // ESP TX → GPS RX
constexpr long GNSS_BAUD = 9600;

/* LiDAR (UART1) — TFMini-Plus or compatible
 *   GPIO 38 = onboard RGB LED — avoid! Using 47/48 instead.
 */
constexpr int LIDAR_RX = 47;  // ESP RX ← LiDAR TX
constexpr int LIDAR_TX = 48;  // ESP TX → LiDAR RX
constexpr long LIDAR_BAUD = 115200;

/* Rotary encoder (KY-040 or similar)
 *   Replaces the BOOT-button ping.  Rotate → change HUD page.
 *   Long-press (2.5 s) → trigger ping.
 */
#define HAS_ENCODER 1
constexpr int ENC_CLK_PIN = 4;         // Channel A (CLK)
constexpr int ENC_DT_PIN  = 5;         // Channel B (DT)
constexpr int ENC_SW_PIN  = 7;         // Push-button (active-LOW)
constexpr unsigned long ENC_LONG_PRESS_MS = 2500;  // hold duration for ping

// Ping button (legacy — kept as fallback if encoder absent)
constexpr int PING_BTN_PIN = 0;        // active-LOW (built-in BOOT btn)
constexpr unsigned long DEBOUNCE_MS = 250;

/* Battery ADC
 *   GPIO 33-37 are used by Octal PSRAM on N32R16V.
 *   GPIO 4 = ADC1_CH3 — safe and accessible on the header.
 */
constexpr int BATT_ADC_PIN = 6;        // voltage divider mid-point
constexpr float BATT_VDIV_RATIO = 2.0; // R1 = R2 → ×2
constexpr float BATT_FULL_V = 4.2;
constexpr float BATT_EMPTY_V = 3.3;

// HUD display — SSD1306 I²C OLED 128×64
constexpr int OLED_WIDTH  = 128;
constexpr int OLED_HEIGHT = 64;
constexpr int OLED_ADDR   = 0x3C;  // common I²C address for SSD1306
// Uses the same I²C bus as IMU/Baro (SDA/SCL above)

// BLE
constexpr const char* BLE_DEVICE_NAME = "LINK-HUD";

// 128-bit UUIDs — must match hud-app/src/lib/ble.ts exactly
constexpr const char* BLE_SERVICE_UUID        = "4c494e4b-4855-4400-b000-000000000000";
constexpr const char* BLE_TELEMETRY_CHAR_UUID = "4c494e4b-4855-4400-b000-000000000001";
constexpr const char* BLE_PIN_CHAR_UUID       = "4c494e4b-4855-4400-b000-000000000002";
constexpr const char* BLE_ACK_CHAR_UUID       = "4c494e4b-4855-4400-b000-000000000003";
constexpr const char* BLE_CAL_CHAR_UUID       = "4c494e4b-4855-4400-b000-000000000004";

// Timing
constexpr unsigned long TELEMETRY_INTERVAL_MS = 200;   // 5 Hz streaming
constexpr unsigned long QUEUE_RETRY_INTERVAL_MS = 10000; // retry unsent pins every 10s
constexpr unsigned long SENSOR_READ_INTERVAL_MS = 50;   // 20 Hz sensor polling
constexpr int MAX_LOCAL_QUEUE = 32;                      // max queued pins on ESP32

// Waypoint computation
constexpr float EARTH_RADIUS_M = 6371000.0f;

}  // namespace cfg
