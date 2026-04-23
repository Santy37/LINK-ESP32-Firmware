/* ══════════════════════════════════════════════════════════════════════
 *  L.I.N.K. — Rotary Encoder Driver (KY-040 / EC11)
 *
 *  Rotation detected via interrupt on CLK pin.
 *  Button long-press (2.5 s) detected via polling.
 * ══════════════════════════════════════════════════════════════════════ */
#pragma once
#include <cstdint>

void encoder_init();

/* Call every loop iteration — updates button state machine.
   Returns true once when a long-press fires. */
bool encoder_poll();

/* Returns accumulated rotation clicks since last call (positive = CW, negative = CCW).
   Resets the counter after reading. */
int encoder_getRotation();
