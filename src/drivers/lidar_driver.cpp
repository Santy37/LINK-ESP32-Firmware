/* LiDAR Driver — PTYS-12X (UART, library-free)

   Protocol (8-byte frames, both directions):
     [0]    0x55         sync
     [1]    0xAA         sync
     [2]    cmd          command byte (0x88 single, 0x89 1Hz, 0xB9 10Hz,
                                       0xC9 100Hz, 0xF9 max, 0x8E stop)
     [3]    status       1 = OK, 0 = no target / out of range
     [4]    reserved
     [5..6] data         distance, big-endian; meters = data / 10.0
     [7]    checksum     RX: sum of bytes [0..6] & 0xFF
                          TX: sum of bytes [2..6] & 0xFF  (header excluded)

   Design notes:
     • lidar_read() is called every loop tick by main.cpp. It is fully
       non-blocking: it drains whatever bytes UART1 has buffered, updates
       a cached LidarData, and returns immediately.
     • Sliding-window parser — every new byte shifts the 8-byte window
       left by one, so a stray 0x55/0xAA inside noise cannot permanently
       desync the parser.
     • Baud is autodetected once at init by actively poking the sensor
       with a single-measure command and listening for a valid frame.
     • Liveness watchdog: if no valid frame arrives for >2s while we
       expected continuous data, re-issue the continuous-mode command.
 */

#include "config.h"
#if HAS_LIDAR

#include "lidar_driver.h"
#include <Arduino.h>
#include <HardwareSerial.h>

namespace {

constexpr uint8_t  PTYS_SYNC0      = 0x55;
constexpr uint8_t  PTYS_SYNC1      = 0xAA;
constexpr uint8_t  CMD_SINGLE      = 0x88;
constexpr uint8_t  CMD_CONT_1HZ    = 0x89;
constexpr uint8_t  CMD_CONT_10HZ   = 0xB9;
constexpr uint8_t  CMD_CONT_100HZ  = 0xC9;
constexpr uint8_t  CMD_STOP        = 0x8E;

// Pick 10 Hz so the firmware's 20 Hz main loop has fresh data ~every other tick.
constexpr uint8_t  CONT_MODE_CMD   = CMD_CONT_10HZ;

// Baud rates the sensor ships with from the factory / after re-flashes.
constexpr uint32_t BAUD_CANDIDATES[] = {115200, 9600, 38400, 57600};
constexpr size_t   BAUD_COUNT        = sizeof(BAUD_CANDIDATES) / sizeof(BAUD_CANDIDATES[0]);

// Watchdog: if no valid frame in this many ms, re-arm continuous mode.
constexpr uint32_t REARM_TIMEOUT_MS  = 2000;

HardwareSerial lidarSerial(1);    // UART1

bool      _initialised   = false;
uint32_t  _lockedBaud    = 0;
uint32_t  _lastFrameMs   = 0;
uint32_t  _lastRearmMs   = 0;
uint32_t  _framesParsed  = 0;
uint32_t  _framesBadCs   = 0;

// Sliding-window parser state — persists across lidar_read() calls so
// frames that arrive split across loop iterations still parse correctly.
uint8_t   _win[8]        = {0};
size_t    _winFilled     = 0;

// Cached latest reading (returned by lidar_read()).
LidarData _cached{};

uint8_t checksumSend(const uint8_t *f) {
  uint16_t s = 0;
  for (int i = 2; i <= 6; i++) s += f[i];
  return (uint8_t)(s & 0xFF);
}

uint8_t checksumRecv(const uint8_t *f) {
  uint16_t s = 0;
  for (int i = 0; i <= 6; i++) s += f[i];
  return (uint8_t)(s & 0xFF);
}

void sendFrame(uint8_t cmd) {
  uint8_t f[8] = {PTYS_SYNC0, PTYS_SYNC1, cmd, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
  f[7] = checksumSend(f);
  lidarSerial.write(f, sizeof(f));
  lidarSerial.flush();
}

bool isRangingCmd(uint8_t c) {
  return c == CMD_SINGLE || c == CMD_CONT_1HZ || c == CMD_CONT_10HZ ||
         c == CMD_CONT_100HZ || c == 0xF9;
}

// Try a single baud rate by poking the sensor and listening briefly.
// Returns true on first valid frame seen.
bool probeBaud(uint32_t baud, uint32_t windowMs) {
  lidarSerial.end();
  delay(50);
  lidarSerial.begin(baud, SERIAL_8N1, cfg::LIDAR_RX, cfg::LIDAR_TX);
  delay(80);

  sendFrame(CMD_SINGLE);

  uint8_t buf[8] = {0};
  size_t  filled = 0;
  uint32_t start = millis();
  while (millis() - start < windowMs) {
    while (lidarSerial.available() > 0) {
      uint8_t b = (uint8_t)lidarSerial.read();
      for (int i = 0; i < 7; i++) buf[i] = buf[i + 1];
      buf[7] = b;
      if (filled < 8) { filled++; continue; }
      if (buf[0] == PTYS_SYNC0 && buf[1] == PTYS_SYNC1 &&
          checksumRecv(buf) == buf[7]) {
        return true;
      }
    }
    delay(2);
  }
  return false;
}

uint32_t autodetectBaud() {
  for (size_t i = 0; i < BAUD_COUNT; i++) {
    Serial.printf("[LIDAR] probing baud %lu ...\n",
                  (unsigned long)BAUD_CANDIDATES[i]);
    if (probeBaud(BAUD_CANDIDATES[i], 700)) {
      return BAUD_CANDIDATES[i];
    }
  }
  return 0;
}

void armContinuousMode() {
  sendFrame(CMD_STOP);
  delay(80);
  sendFrame(CONT_MODE_CMD);
  _lastRearmMs = millis();
}

// Drain UART, run sliding-window parser, update _cached on any good frame.
void drainAndParse() {
  while (lidarSerial.available() > 0) {
    uint8_t b = (uint8_t)lidarSerial.read();
    for (int i = 0; i < 7; i++) _win[i] = _win[i + 1];
    _win[7] = b;
    if (_winFilled < 8) { _winFilled++; continue; }

    // Cheap header check first to avoid checksumming every byte position.
    if (_win[0] != PTYS_SYNC0 || _win[1] != PTYS_SYNC1) continue;

    if (checksumRecv(_win) != _win[7]) {
      _framesBadCs++;
      continue;
    }

    // Header + checksum valid — accept the frame.
    uint8_t  cmd    = _win[2];
    uint8_t  status = _win[3];
    uint16_t raw    = ((uint16_t)_win[5] << 8) | _win[6];

    _framesParsed++;
    _lastFrameMs = millis();

    if (isRangingCmd(cmd)) {
      // Distance: PTYS-12X reports decimeters → meters = raw / 10.0
      // If your unit reports centimeters, change to raw / 100.0
      float meters = raw / 10.0f;

      _cached.rangeM  = meters;
      _cached.quality = (status == 1) ? 255 : 0;
      _cached.valid   = (status == 1) && (raw > 0) && (meters < 100.0f);
      _cached.state   = _cached.valid ? ModuleState::OK : ModuleState::DEGRADED;
    }
    // ack frames for STOP / config commands: ignore payload, just update liveness.

    // Consume the matched frame so it can't re-match next byte.
    _winFilled = 0;
    for (int i = 0; i < 8; i++) _win[i] = 0;
  }
}

}  // namespace

bool lidar_init() {
  Serial.println("[LIDAR] PTYS-12X init: auto-detecting baud...");

  uint32_t baud = autodetectBaud();
  if (baud == 0) {
    Serial.printf("[LIDAR] autodetect failed, falling back to %ld\n",
                  cfg::LIDAR_BAUD);
    baud = cfg::LIDAR_BAUD;
    lidarSerial.end();
    delay(50);
    lidarSerial.begin(baud, SERIAL_8N1, cfg::LIDAR_RX, cfg::LIDAR_TX);
    _initialised = false;
  } else {
    Serial.printf("[LIDAR] locked baud %lu\n", (unsigned long)baud);
    _initialised = true;
  }
  _lockedBaud = baud;

  // Start streaming.
  armContinuousMode();

  // Reset cache to a defined state so first lidar_read() doesn't return junk.
  _cached = LidarData{};
  _cached.state = _initialised ? ModuleState::DEGRADED : ModuleState::FAIL;
  _lastFrameMs  = 0;
  _winFilled    = 0;

  return _initialised;
}

bool lidar_selfTest() {
  if (!_initialised) return false;

  // Give the sensor up to 500 ms to deliver one valid ranging frame.
  uint32_t start = millis();
  while (millis() - start < 500) {
    drainAndParse();
    if (_framesParsed > 0 && _lastFrameMs >= start) break;
    delay(10);
  }

  bool ok = _framesParsed > 0;
  Serial.printf("[LIDAR] self-test parsed=%lu badCs=%lu range=%.2fm valid=%d → %s\n",
                (unsigned long)_framesParsed,
                (unsigned long)_framesBadCs,
                _cached.rangeM,
                _cached.valid,
                ok ? "PASS" : "FAIL");
  return ok;
}

LidarData lidar_read() {
  if (!_initialised) {
    LidarData d{};
    d.state = ModuleState::FAIL;
    return d;
  }

  drainAndParse();

  // Liveness watchdog: if continuous mode silently stopped (sensor reset,
  // brown-out, glitched config), nudge it back on.
  uint32_t now = millis();
  if (_lastFrameMs != 0 && (now - _lastFrameMs) > REARM_TIMEOUT_MS) {
    if ((now - _lastRearmMs) > REARM_TIMEOUT_MS) {
      Serial.println("[LIDAR] no frames for >2s — re-arming continuous mode");
      armContinuousMode();
      _cached.valid = false;
      _cached.state = ModuleState::DEGRADED;
    }
  }

  return _cached;
}

#endif // HAS_LIDAR
