/* BLE GATT Server — advertises LINK-HUD service
 
 Service UUID : 4c494e4b-4855-4400-b000-000000000000
 Characteristics:
   0x0001  Telemetry  (Notify)   — periodic JSON telemetry stream
   0x0002  PinPayload (Notify)   — sent on ping button press
   0x0003  ACK        (Write)    — phone writes { ack, pinId }
 
 Matches hud-app/src/lib/ble.ts on the phone side.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <Arduino.h>

// Callback invoked when the phone writes an ACK for a pin id.
using AckCallback = std::function<void(const String& pinId)>;

// Callback invoked when a phone connects / disconnects.
using ConnCallback = std::function<void(bool connected)>;

// Callback invoked when the phone writes a QNH (sea-level pressure) value.
using CalCallback = std::function<void(float qnhHPa)>;

// Public API

// Initialise BLE stack, create GATT service, start advertising.
void ble_init();

// Start / restart BLE advertising (called after disconnect too).
void ble_startAdvertising();

// Returns true when a phone is connected.
bool ble_isConnected();

// Send a JSON telemetry blob via Notify on the telemetry characteristic.
void ble_sendTelemetry(const String& json);

// Send a JSON pin-payload blob via Notify on the pin characteristic.
void ble_sendPinPayload(const String& json);

// Register callback for ACKs received from the phone.
void ble_onAck(AckCallback cb);

// Register callback for connection state changes.
void ble_onConnection(ConnCallback cb);

// Register callback for calibration (QNH) writes from the phone.
void ble_onCalibration(CalCallback cb);
