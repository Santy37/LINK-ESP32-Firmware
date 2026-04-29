// BLE GATT Server implementation (NimBLE / Arduino BLE)

#include "ble_server.h"
#include "config.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// State
static BLEServer*         pServer         = nullptr;
static BLECharacteristic* pTelemetryChar  = nullptr;
static BLECharacteristic* pPinChar        = nullptr;
static BLECharacteristic* pAckChar        = nullptr;
static BLECharacteristic* pCalChar        = nullptr;
static bool               _deviceConnected = false;
static AckCallback        _ackCb           = nullptr;
static ConnCallback       _connCb          = nullptr;
static CalCallback        _calCb           = nullptr;

// Server callbacks

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* /*s*/) override {
    _deviceConnected = true;
    Serial.println("[BLE] phone connected");
    if (_connCb) _connCb(true);
  }

  void onDisconnect(BLEServer* /*s*/) override {
    _deviceConnected = false;
    Serial.println("[BLE] phone disconnected — re-advertising…");
    if (_connCb) _connCb(false);
    ble_startAdvertising();
  }
};

// ACK characteristic write callback

class AckWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string raw = pChar->getValue();
    if (raw.length() == 0) return;   // empty write — ignore

    Serial.printf("[BLE] ACK received: %s\n", raw.c_str());

    // Parse { "ack": true, "pinId": "..." }
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, raw);
    if (err) {
      // Phone sent garbage — just log and move on, don't crash the BLE stack
      Serial.printf("[BLE] ACK parse error: %s\n", err.c_str());
      return;
    }

    // Hand the pinId off to main — main marks it ACK'd in the queue
    if (doc["ack"].as<bool>() && doc.containsKey("pinId")) {
      String pinId = doc["pinId"].as<String>();
      if (_ackCb) _ackCb(pinId);
    }
  }
};

// Calibration characteristic write callback: { "qnhHPa": 1020.3 }
// Phone pulls QNH (sea-level pressure) from a weather API for the user's
// current location and pushes it down so our barometer reads true MSL.
class CalWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string raw = pChar->getValue();
    if (raw.length() == 0) return;

    Serial.printf("[BLE] CAL received: %s\n", raw.c_str());

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, raw);
    if (err) {
      Serial.printf("[BLE] CAL parse error: %s\n", err.c_str());
      return;
    }

    // Sanity-check the value before passing it to the baro — anything
    // outside the 800–1100 hPa range is either bogus or a hurricane,
    // either way we don't want it touching the calibration
    if (doc.containsKey("qnhHPa")) {
      float q = doc["qnhHPa"].as<float>();
      if (q > 800.0f && q < 1100.0f && _calCb) {
        _calCb(q);
      }
    }
  }
};

// Public API

void ble_init() {
  BLEDevice::init(cfg::BLE_DEVICE_NAME);
  BLEDevice::setMTU(512);                          // request large MTU for JSON

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(cfg::BLE_SERVICE_UUID);

  // Telemetry: Notify
  pTelemetryChar = pService->createCharacteristic(
    cfg::BLE_TELEMETRY_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTelemetryChar->addDescriptor(new BLE2902());     // CCCD for notify

  // Pin Payload: Notify
  pPinChar = pService->createCharacteristic(
    cfg::BLE_PIN_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pPinChar->addDescriptor(new BLE2902());

  // ACK: Write
  pAckChar = pService->createCharacteristic(
    cfg::BLE_ACK_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pAckChar->setCallbacks(new AckWriteCallback());

  // Calibration: Write (QNH sea-level pressure from phone weather API)
  pCalChar = pService->createCharacteristic(
    cfg::BLE_CAL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCalChar->setCallbacks(new CalWriteCallback());

  pService->start();
  ble_startAdvertising();

  Serial.printf("[BLE] GATT service started — advertising as \"%s\"\n",
                cfg::BLE_DEVICE_NAME);
}

void ble_startAdvertising() {
  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(cfg::BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);  // helps iPhone find it
  BLEDevice::startAdvertising();
}

bool ble_isConnected() {
  return _deviceConnected;
}

void ble_sendTelemetry(const String& json) {
  if (!_deviceConnected || !pTelemetryChar) return;
  pTelemetryChar->setValue(json.c_str());
  pTelemetryChar->notify();
}

void ble_sendPinPayload(const String& json) {
  if (!_deviceConnected || !pPinChar) return;
  pPinChar->setValue(json.c_str());
  pPinChar->notify();
}

void ble_onAck(AckCallback cb) {
  _ackCb = cb;
}

void ble_onConnection(ConnCallback cb) {
  _connCb = cb;
}

void ble_onCalibration(CalCallback cb) {
  _calCb = cb;
}
