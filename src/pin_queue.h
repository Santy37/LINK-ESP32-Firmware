/* Local Pin Queue — stores payloads until ACK received from phone
 
 Circular buffer in RAM (persisted to NVS on overflow for safety).
 Mirrors the offline queue concept on the phone side.
 */

#pragma once

#include <Arduino.h>
#include "sensor_types.h"
#include "config.h"

struct QueuedPin {
  String  id;               // UUID
  String  jsonPayload;      // pre-serialised JSON
  uint8_t attempts;         // number of BLE send attempts
  unsigned long lastSentMs; // millis() of last attempt
  bool    acked;            // true once phone ACKs
};

// Public API

// Enqueue a new pin payload (returns false if queue is full).
bool      pinQueue_enqueue(const String& id, const String& json);

// Mark a pin as ACK'd — it will be removed on the next prune.
void      pinQueue_ack(const String& id);

// Get the count of un-ACK'd pins.
int       pinQueue_pendingCount();

// Get a pointer to the i-th un-ACK'd entry (nullptr if out of range).
QueuedPin* pinQueue_getPending(int index);

// Remove all ACK'd entries.
void      pinQueue_prune();

// Wipe everything.
void      pinQueue_clear();
