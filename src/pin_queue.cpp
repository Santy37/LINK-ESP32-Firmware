// Local Pin Queue — implementation

#include "pin_queue.h"

static QueuedPin _queue[cfg::MAX_LOCAL_QUEUE];
static int       _count = 0;

bool pinQueue_enqueue(const String& id, const String& json) {
  if (_count >= cfg::MAX_LOCAL_QUEUE) {
    Serial.println("[QUEUE] full — dropping oldest un-ACK'd pin");
    // Drop the oldest un-ACK'd entry
    for (int i = 0; i < _count; i++) {
      if (!_queue[i].acked) {
        // Shift everything down
        for (int j = i; j < _count - 1; j++) {
          _queue[j] = _queue[j + 1];
        }
        _count--;
        break;
      }
    }
    if (_count >= cfg::MAX_LOCAL_QUEUE) return false;
  }

  QueuedPin& e = _queue[_count++];
  e.id          = id;
  e.jsonPayload = json;
  e.attempts    = 0;
  e.lastSentMs  = 0;
  e.acked       = false;

  Serial.printf("[QUEUE] enqueued pin %s  (queue size: %d)\n", id.c_str(), _count);
  return true;
}

void pinQueue_ack(const String& id) {
  for (int i = 0; i < _count; i++) {
    if (_queue[i].id == id) {
      _queue[i].acked = true;
      Serial.printf("[QUEUE] ACK for pin %s — marked delivered\n", id.c_str());
      return;
    }
  }
  Serial.printf("[QUEUE] ACK for unknown pin %s (already pruned?)\n", id.c_str());
}

int pinQueue_pendingCount() {
  int n = 0;
  for (int i = 0; i < _count; i++) {
    if (!_queue[i].acked) n++;
  }
  return n;
}

QueuedPin* pinQueue_getPending(int index) {
  int seen = 0;
  for (int i = 0; i < _count; i++) {
    if (!_queue[i].acked) {
      if (seen == index) return &_queue[i];
      seen++;
    }
  }
  return nullptr;
}

void pinQueue_prune() {
  int write = 0;
  for (int read = 0; read < _count; read++) {
    if (!_queue[read].acked) {
      if (write != read) _queue[write] = _queue[read];
      write++;
    }
  }
  int removed = _count - write;
  _count = write;
  if (removed > 0) {
    Serial.printf("[QUEUE] pruned %d ACK'd entries (remaining: %d)\n", removed, _count);
  }
}

void pinQueue_clear() {
  _count = 0;
  Serial.println("[QUEUE] cleared");
}
