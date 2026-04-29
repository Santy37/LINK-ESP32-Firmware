// Local Pin Queue — implementation

#include "pin_queue.h"

static QueuedPin _queue[cfg::MAX_LOCAL_QUEUE];
static int       _count = 0;

bool pinQueue_enqueue(const String& id, const String& json) {
  // Queue full? Make room by evicting the oldest un-ACK'd entry.
  // ACK'd entries should already be pruned, but defensive check anyway.
  if (_count >= cfg::MAX_LOCAL_QUEUE) {
    Serial.println("[QUEUE] full — dropping oldest un-ACK'd pin");
    for (int i = 0; i < _count; i++) {
      if (!_queue[i].acked) {
        // Shift everything after i down by one slot to fill the gap
        for (int j = i; j < _count - 1; j++) {
          _queue[j] = _queue[j + 1];
        }
        _count--;
        break;   // only drop one — we just need one slot
      }
    }
    // Still no room? Then literally everything is ACK'd waiting to prune.
    // Bail — caller can retry after the next prune pass.
    if (_count >= cfg::MAX_LOCAL_QUEUE) return false;
  }

  // Stick the new pin at the end (newest = highest index)
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
  // Two-pointer compact: walk through with `read`, copy survivors
  // (the un-ACK'd ones) down to `write`. Skips the gaps left by
  // ACK'd pins without needing a second buffer.
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
