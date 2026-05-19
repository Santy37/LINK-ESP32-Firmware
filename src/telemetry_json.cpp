// Telemetry Serialiser — implementation

#include "telemetry_json.h"
#include <ArduinoJson.h>

// Helper: convert ModuleState enum to the string expected by the phone.
const char* moduleStateStr(ModuleState s) {
  switch (s) {
    case ModuleState::OK:       return "ok";
    case ModuleState::DEGRADED: return "degraded";
    case ModuleState::FAIL:     return "fail";
  }
  return "fail";
}

const char* gnssFixStr(GnssFix f) {
  switch (f) {
    case GnssFix::NONE:   return "none";
    case GnssFix::FIX_2D: return "2d";
    case GnssFix::FIX_3D: return "3d";
    case GnssFix::DGPS:   return "dgps";
  }
  return "none";
}

// telemetry_toJson

String telemetry_toJson(const TelemetrySnapshot& t) {
  JsonDocument doc;

  // Timestamp (ISO-8601 — ESP32 doesn't have real RTC, so epoch-ish)
  char tsBuf[32];
  unsigned long sec = millis() / 1000;
  snprintf(tsBuf, sizeof(tsBuf), "T+%lu", sec);
  doc["ts"] = tsBuf;

  // IMU
  JsonObject imu = doc["imu"].to<JsonObject>();
  imu["heading"] = serialized(String(t.imu.heading, 1));
  imu["pitch"]   = serialized(String(t.imu.pitch, 1));
  imu["roll"]    = serialized(String(t.imu.roll, 1));

  // GNSS
  JsonObject gnss = doc["gnss"].to<JsonObject>();
  gnss["lat"]  = serialized(String(t.gnss.lat, 7));
  gnss["lon"]  = serialized(String(t.gnss.lon, 7));
  gnss["altM"] = serialized(String(t.gnss.altM, 1));
  gnss["accM"] = serialized(String(t.gnss.accM, 1));
  gnss["fix"]  = gnssFixStr(t.gnss.fix);
  gnss["sats"] = t.gnss.sats;

  // Baro
  JsonObject baro = doc["baro"].to<JsonObject>();
  baro["pressHPa"] = serialized(String(t.baro.pressHPa, 1));
  baro["tempC"]    = serialized(String(t.baro.tempC, 1));
  baro["altEstM"]  = serialized(String(t.baro.altEstM, 1));

  // LiDAR
  JsonObject lidar = doc["lidar"].to<JsonObject>();
  lidar["rangeM"]  = serialized(String(t.lidar.rangeM, 2));
  lidar["quality"] = t.lidar.quality;
  lidar["valid"]   = t.lidar.valid;

  // Battery
  doc["battery"] = t.battery;

  // Module status
  JsonObject mods = doc["modules"].to<JsonObject>();
  mods["imu"]   = moduleStateStr(t.imu.state);
  mods["gnss"]  = moduleStateStr(t.gnss.state);
  mods["baro"]  = moduleStateStr(t.baro.state);
  mods["lidar"] = moduleStateStr(t.lidar.state);
  mods["hud"]   = "ok";  // TODO: real HUD health check

  String out;
  serializeJson(doc, out);
  return out;
}

// pinPayload_toJson

String pinPayload_toJson(const String& id,
                          const TelemetrySnapshot& t,
                          double targetLat, double targetLon,
                          const char* label) {
  JsonDocument doc;

  doc["id"]         = id;
  doc["receivedAt"] = 0;  // phone will overwrite this

  // Observer
  JsonObject obs = doc["observer"].to<JsonObject>();
  obs["lat"]  = serialized(String(t.gnss.lat, 7));
  obs["lon"]  = serialized(String(t.gnss.lon, 7));
  obs["altM"] = serialized(String(t.gnss.altM, 1));
  obs["accM"] = serialized(String(t.gnss.accM, 1));

  // Target
  JsonObject tgt = doc["target"].to<JsonObject>();
  tgt["lat"]     = serialized(String(targetLat, 7));
  tgt["lon"]     = serialized(String(targetLon, 7));
  tgt["altEstM"] = serialized(String(t.baro.altEstM, 1));

  // Aiming
  JsonObject aim = doc["aiming"].to<JsonObject>();
  aim["bearingDeg"]   = serialized(String(t.imu.heading, 1));
  aim["pitchDeg"]     = serialized(String(t.imu.pitch, 1));
  aim["rangeM"]       = serialized(String(t.lidar.rangeM, 2));
  aim["lidarQuality"] = t.lidar.quality;

  doc["label"] = label;

  // Slim sensor-health summary so the server can record which modules were
  // healthy at ping time. We deliberately do NOT embed the full telemetry
  // snapshot — that blew past the BLE characteristic 600-byte indication
  // cap and caused all pins to fail with "Size 6xx too large". Observer /
  // target / aiming above carries the actual measurements.
  JsonObject mods = doc["modules"].to<JsonObject>();
  mods["imu"]   = moduleStateStr(t.imu.state);
  mods["gnss"]  = moduleStateStr(t.gnss.state);
  mods["baro"]  = moduleStateStr(t.baro.state);
  mods["lidar"] = moduleStateStr(t.lidar.state);

  String out;
  serializeJson(doc, out);
  return out;
}
