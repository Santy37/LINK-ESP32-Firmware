/* Rotary Encoder Driver (KY-040 / EC11)
 Rotation comes in via a CLK-pin interrupt — fast path, no polling cost.
 Button long-press (2.5 s) is detected by polling in encoder_poll() since
 we don't need sub-millisecond accuracy on a "hold the knob" gesture.
 */
#pragma once
#include <cstdint>

void encoder_init();

// Call every loop iteration — runs the button state machine.
// Returns true exactly once on the rising edge of a long-press firing.
bool encoder_poll();

// Returns accumulated rotation clicks since the last call.
// Positive = CW, negative = CCW. Resets the internal counter after reading,
// so callers don't have to track deltas themselves.
int encoder_getRotation();
