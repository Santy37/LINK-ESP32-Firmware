/* LiDAR Driver — TFMini-Plus (UART)
 *
 * Reads distance and signal quality from the TFMini-Plus over UART1.
 * Protocol: 9-byte frames, header 0x59 0x59.
 */

#include "config.h"
#if HAS_LIDAR

#include "lidar_driver.h"
#include <HardwareSerial.h>
#include <algorithm>

static HardwareSerial lidarSerial(1);   // UART1
static bool _initialised = false;

bool lidar_init() {
  lidarSerial.begin(cfg::LIDAR_BAUD, SERIAL_8N1, cfg::LIDAR_RX, cfg::LIDAR_TX);
  delay(100);

  // Try to read a frame within 500ms
  unsigned long start = millis();
  while (millis() - start < 500) {
    if (lidarSerial.available() >= 9) {
      _initialised = true;
      Serial.println("[LIDAR] TFMini data stream detected — OK");
      return true;
    }
    delay(10);
  }

  Serial.println("[LIDAR] no data within 500ms — FAIL");
  _initialised = false;
  return false;
}

bool lidar_selfTest() {
  if (!_initialised) return false;

  LidarData d = lidar_read();
  bool ok = d.valid && d.rangeM > 0.01f && d.rangeM < 12000.0f;
  Serial.printf("[LIDAR] self-test range=%.2fm qual=%d valid=%d → %s\n",
                d.rangeM, d.quality, d.valid, ok ? "PASS" : "FAIL");
  return ok;
}

LidarData lidar_read() {
  LidarData d{};
  d.valid = false;
  d.state = _initialised ? ModuleState::DEGRADED : ModuleState::FAIL;

  if (!_initialised) return d;

  // Scan for a valid 9-byte TFMini frame
  int attempts = 0;
  while (lidarSerial.available() >= 9 && attempts < 20) {
    attempts++;

    uint8_t header1 = lidarSerial.read();
    if (header1 != 0x59) continue;

    uint8_t header2 = lidarSerial.peek();
    if (header2 != 0x59) continue;
    lidarSerial.read();  // consume header2

    uint8_t buf[7];
    lidarSerial.readBytes(buf, 7);

    uint16_t dist     = buf[0] | (buf[1] << 8);   // cm
    uint16_t strength = buf[2] | (buf[3] << 8);

    // Checksum: sum of all 8 bytes before checksum
    uint8_t ck = (0x59 + 0x59);
    for (int i = 0; i < 6; i++) ck += buf[i];
    if (ck != buf[6]) continue;  // bad checksum, try next frame

    d.rangeM  = dist / 100.0f;            // cm → m
    d.quality = (uint8_t)std::min((int)strength, 255);
    d.valid   = (dist > 0 && strength > 50);

    d.state = d.valid ? ModuleState::OK : ModuleState::DEGRADED;
    return d;
  }

  return d;  // no valid frame found
}

#endif // HAS_LIDAR
