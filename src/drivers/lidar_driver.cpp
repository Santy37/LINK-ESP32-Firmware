/* LiDAR Driver — PTYS-12X (UART, library-free)

   Protocol (8-byte frames, both directions) — verified against JRT
   PTYS-12X datasheet §4.1–4.3:
     [0]    0x55         sync
     [1]    0xAA         sync
     [2]    cmd / Freq   0x88 single, 0x8E stop,
                        0x89 cont@1Hz, 0xB9 cont@5Hz, 0xC9 cont@10Hz,
                        0xF9 axis-calibration mode (NOT a ranging rate)
     [3]    status       1 = OK, 0 = no target / out of range
     [4]    reserved (0xFF on TX, 0xFF on RX)
     [5..6] data         distance × 10, big-endian (meters = raw / 10.0)
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
constexpr uint8_t  CMD_CONT_5HZ    = 0xB9;   // datasheet §4.2
constexpr uint8_t  CMD_CONT_10HZ   = 0xC9;   // datasheet §4.2
constexpr uint8_t  CMD_CALIB       = 0xF9;   // axis-calibration, not a ranging rate
constexpr uint8_t  CMD_STOP        = 0x8E;

// Pick 10 Hz so the firmware's 20 Hz main loop has fresh data ~every other tick.
// At long range on low-reflectivity targets, dropping to CMD_CONT_1HZ gives
// the sensor 10× the integration time per measurement — noticeable SNR
// improvement on weak returns. Datasheet §2 actually notes "single
// application: 1 Hz; close range: 2/3/4 Hz meet functional requirements",
// implying 1 Hz is the sweet spot for long-range work.
constexpr uint8_t  CONT_MODE_CMD   = CMD_CONT_10HZ;

// Baud rates the sensor ships with from the factory / after re-flashes.
constexpr uint32_t BAUD_CANDIDATES[] = {115200, 9600, 38400, 57600};
constexpr size_t   BAUD_COUNT        = sizeof(BAUD_CANDIDATES) / sizeof(BAUD_CANDIDATES[0]);

// Watchdog: if no valid frame in this many ms, re-arm continuous mode.
// Kept generous so pointing at empty sky (no return target → some PTYS
// firmwares stop transmitting until they re-acquire) doesn't trigger a
// re-arm storm that blinks the laser off in an IR viewer.
constexpr uint32_t REARM_TIMEOUT_MS  = 5000;

// Sensor power-on boot time. The PTYS-12X internal MCU needs roughly
// 500–800 ms after VCC rises before it will accept UART commands — but on
// a cold boot from a fully-discharged input cap, the 5 V rail ramps slowly
// and the sensor's own brown-out recovery can push that to >1.5 s. The
// ESP32-S3 boots much faster than that, so without a generous settle the
// first autodetect window misses every cold start.
constexpr uint32_t SENSOR_BOOT_MS    = 1800;

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
  return c == CMD_SINGLE  || c == CMD_CONT_1HZ ||
         c == CMD_CONT_5HZ || c == CMD_CONT_10HZ ||
         c == CMD_CALIB;
}

// Try a single baud rate by kicking the sensor into continuous mode and
// listening briefly. We deliberately use CMD_CONT_10HZ instead of
// CMD_SINGLE here: the cold-start failure mode on this PTYS-12X variant
// is that CMD_SINGLE produces no reply when the sensor boots into idle,
// while CMD_CONT_10HZ reliably starts a frame stream. (Post-soft-reset
// the sensor is already streaming from the previous run, which is why
// hitting RST always made autodetect succeed instantly — we were just
// catching the leftover stream, not actually getting a CMD_SINGLE reply.)
bool probeBaud(uint32_t baud, uint32_t windowMs) {
  lidarSerial.end();
  delay(50);
  lidarSerial.begin(baud, SERIAL_8N1, cfg::LIDAR_RX, cfg::LIDAR_TX);
  delay(80);

  // Drain any boot-time noise so it can't masquerade as a frame header.
  while (lidarSerial.available()) lidarSerial.read();

  sendFrame(CONT_MODE_CMD);

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
  // Try the configured baud first — on a stable installation we know which
  // baud the sensor uses, so checking it first turns the common case into
  // a single short probe instead of cycling through all four candidates.
  Serial.printf("[LIDAR] probing configured baud %lu ...\n",
                (unsigned long)cfg::LIDAR_BAUD);
  if (probeBaud(cfg::LIDAR_BAUD, 1200)) return cfg::LIDAR_BAUD;

  for (size_t i = 0; i < BAUD_COUNT; i++) {
    if (BAUD_CANDIDATES[i] == cfg::LIDAR_BAUD) continue;  // already tried
    Serial.printf("[LIDAR] probing baud %lu ...\n",
                  (unsigned long)BAUD_CANDIDATES[i]);
    if (probeBaud(BAUD_CANDIDATES[i], 700)) {
      return BAUD_CANDIDATES[i];
    }
  }
  return 0;
}

void armContinuousMode() {
  // NOTE: we deliberately do NOT send CMD_STOP here. Sending STOP physically
  // halts the laser for ~80 ms, which is visible as the IR LED blinking off
  // in an IR viewer. The PTYS-12X happily accepts a new continuous-mode
  // command without a preceding stop — it just replaces the active rate.
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
      // Trust the sensor's status byte rather than imposing a software
      // range cap — PTYS-12X variants legitimately report out to ~150 m
      // on high-reflectivity targets. Lower bound stays at >0 to reject
      // the "no target" placeholder (raw=0).
      _cached.valid   = (status == 1) && (raw > 0);
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
  Serial.println("[LIDAR] PTYS-12X init: waiting for sensor boot...");

  // CRITICAL: bring up our UART BEFORE the boot delay so the TX line going
  // to the LiDAR's RX is driven idle-high while the sensor MCU boots.
  // If we don't, that pin floats during the delay, the LiDAR sees noise on
  // its RX during its own boot window, and its UART parser locks up — which
  // is the cold-start "needs reset to work" symptom. (After a soft reset
  // the pin is already idle-high from the previous run, masking the bug.)
  lidarSerial.begin(cfg::LIDAR_BAUD, SERIAL_8N1, cfg::LIDAR_RX, cfg::LIDAR_TX);

  // Give the sensor's internal MCU time to fully boot before poking it.
  // Without this, ~every cold-boot the ESP32 wins the race and autodetect
  // fails until the user hits the reset button.
  delay(SENSOR_BOOT_MS);

  // Drain any garbage bytes accumulated during the LiDAR's boot (e.g. a
  // boot banner or framing errors clocked in while VCC was still ramping).
  while (lidarSerial.available()) lidarSerial.read();

  // Try autodetect up to 5 times, with extra settling between attempts.
  // Most coldboots succeed on attempt 1 after SENSOR_BOOT_MS; the extra
  // retries are insurance against slow-ramping 5 V rails and brown-out
  // recovery on cold starts.
  uint32_t baud = 0;
  for (int attempt = 1; attempt <= 5 && baud == 0; attempt++) {
    Serial.printf("[LIDAR] autodetect attempt %d/5 ...\n", attempt);
    baud = autodetectBaud();
    if (baud == 0 && attempt < 5) {
      Serial.println("[LIDAR] no response — settling 800 ms before retry");
      delay(800);
    }
  }

  if (baud == 0) {
    Serial.printf("[LIDAR] autodetect failed after 5 attempts, falling back to %ld\n",
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

  uint32_t now = millis();

  // The parser cache otherwise remains valid until the 5-second re-arm
  // watchdog. Reject it much sooner for waypoint math while still allowing
  // the longer watchdog to avoid repeatedly restarting the sensor.
  if (_cached.valid &&
      (_lastFrameMs == 0 || (now - _lastFrameMs) > cfg::LIDAR_MAX_SAMPLE_AGE_MS)) {
    _cached.valid = false;
    _cached.state = ModuleState::DEGRADED;
  }

  // Liveness watchdog: if continuous mode silently stopped (sensor reset,
  // brown-out, glitched config), nudge it back on. Threshold is generous
  // (5 s) so normal "pointing at sky / no target" doesn't trigger — that
  // would otherwise blink the laser visible in an IR viewer.
  if (_lastFrameMs != 0 && (now - _lastFrameMs) > REARM_TIMEOUT_MS) {
    if ((now - _lastRearmMs) > REARM_TIMEOUT_MS) {
      Serial.println("[LIDAR] no frames for >5s — re-arming continuous mode");
      armContinuousMode();
      _cached.valid = false;
      _cached.state = ModuleState::DEGRADED;
    }
  }

  return _cached;
}

#endif // HAS_LIDAR
