/* L.I.N.K. ESP32-S3 Firmware — tft_driver.h

   ST7735 1.8" 128×160 RGB TFT — HUD page renderer.

   API mirrors the inline OLED rendering in main.cpp so each
   call site has a 1:1 TFT counterpart. All drawing is done
   per-line with fillRect+drawString to avoid full-screen flicker.
   See platformio.ini for TFT_eSPI build-flag configuration.
 */
#pragma once

#include "config.h"

#if HAS_TFT
#include "sensor_types.h"

namespace tft {

bool init();                                  // returns true on success
bool isReady();

// Boot / status screens (full repaint)
void renderBootSplash();
void renderReadyScreen(bool gnssOk);
void renderPingFail(const char* line2);

// Live HUD pages — repainted at ~2 Hz
void renderPingHold(uint8_t percent);
void renderPingAbort(const char* reason);   // sensor went down mid-hold
void renderPingSent(int queuedPins);        // brief confirmation after success
void renderPage(int hudPage,
                int totalPages,
                const TelemetrySnapshot& snap,
                bool bleConnected,
                int  queuedPins);

}  // namespace tft

#endif  // HAS_TFT
