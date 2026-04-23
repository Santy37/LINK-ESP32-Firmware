// L.I.N.K. — Rotary Encoder Driver (KY-040 / EC11)

/* Uses polling-based quadrature state machine instead of interrupts.
 * Cheap mechanical encoders bounce too much for edge-triggered ISRs.
 * Button long-press (2.5 s) detected via polling.
 */

#include "encoder_driver.h"
#include <Arduino.h>
#include "config.h"

/* Quadrature state machine (polled, no ISR)
 * Gray code sequence for one CW detent:  00 → 01 → 11 → 10 → 00
 * The lookup table maps (prevState << 2 | currState) to direction.
 * +1 = CW step, -1 = CCW step, 0 = same or invalid (bounce).
 */
static const int8_t QUAD_TABLE[16] = {
   0, +1,  -1,  0,
  -1,  0,   0, +1,
  +1,  0,   0, -1,
   0, -1,  +1,  0
};

static uint8_t lastAB    = 0;   // previous 2-bit state (CLK<<1 | DT)
static int8_t  quadSteps = 0;   // accumulated partial steps
static int     rotCount  = 0;   // full-detent count (read by main loop)

/* EC11 typically has 4 Gray-code transitions per detent.
   We only register a click after 4 accumulated same-direction steps. */
static constexpr int8_t STEPS_PER_DETENT = 4;

// Button state machine
static bool     btnDown       = false;
static unsigned long btnDownAt = 0;
static bool     longPressFired = false;


void encoder_init() {
  pinMode(cfg::ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(cfg::ENC_DT_PIN,  INPUT_PULLUP);
  pinMode(cfg::ENC_SW_PIN,  INPUT_PULLUP);

  // Read initial state
  lastAB = (digitalRead(cfg::ENC_CLK_PIN) << 1) | digitalRead(cfg::ENC_DT_PIN);

  Serial.printf("[ENC]  CLK=%d  DT=%d  SW=%d  long-press=%lums (quad poll)\n",
                cfg::ENC_CLK_PIN, cfg::ENC_DT_PIN, cfg::ENC_SW_PIN,
                cfg::ENC_LONG_PRESS_MS);
}

bool encoder_poll() {
  bool pressed = (digitalRead(cfg::ENC_SW_PIN) == LOW);
  unsigned long now = millis();

  if (pressed && !btnDown) {
    // Falling edge — button just pressed
    btnDown        = true;
    btnDownAt      = now;
    longPressFired = false;
  }

  if (!pressed && btnDown) {
    // Rising edge — button released
    btnDown = false;
    // (short-press can be handled here if needed later)
  }

  // Long-press fires once while held
  if (btnDown && !longPressFired &&
      (now - btnDownAt) >= cfg::ENC_LONG_PRESS_MS) {
    longPressFired = true;
    return true;
  }

  return false;
}

int encoder_getRotation() {
  // Sample current pin state
  uint8_t curAB = (digitalRead(cfg::ENC_CLK_PIN) << 1) | digitalRead(cfg::ENC_DT_PIN);

  if (curAB != lastAB) {
    int8_t dir = QUAD_TABLE[(lastAB << 2) | curAB];
    lastAB = curAB;
    quadSteps += dir;

    // One full detent reached?
    if (quadSteps >= STEPS_PER_DETENT) {
      quadSteps = 0;
      rotCount++;
    } else if (quadSteps <= -STEPS_PER_DETENT) {
      quadSteps = 0;
      rotCount--;
    }
  }

  int val = rotCount;
  rotCount = 0;
  return val;
}
