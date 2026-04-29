/* Telemetry Serialiser — converts sensor structs → JSON Strings
 
 Two serialisation functions:
   1. telemetry_toJson()   → periodic 5 Hz stream
   2. pinPayload_toJson()  → one-shot on ping button press
 
 Output matches hud-app/src/lib/payloadTypes.ts exactly.
 */

#pragma once

#include <Arduino.h>
#include "sensor_types.h"

/*
 Serialise a TelemetrySnapshot to compact JSON.
 Output ≈ 350-450 bytes — fits in a single BLE notification with MTU 512.
 */
String telemetry_toJson(const TelemetrySnapshot& t);

/*
 Build the full PinPayload JSON sent on a ping button press.
 Includes observer, computed target, aiming data, and the full telemetry.
 */
String pinPayload_toJson(const String& id,
                          const TelemetrySnapshot& t,
                          double targetLat, double targetLon,
                          const char* label = "Waypoint");
