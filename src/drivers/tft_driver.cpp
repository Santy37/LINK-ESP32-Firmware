/* L.I.N.K. ESP32-S3 Firmware — tft_driver.cpp

   ST7735 1.8" 128×160 RGB TFT renderer for the HUD.

   Design notes
   ────────────
   • TFT_eSPI is configured entirely through build_flags in
     platformio.ini; no User_Setup.h is read (USER_SETUP_LOADED=1).
   • Screen is held at 128 × 160 portrait by default — change
     cfg::TFT_ROTATION in include/config.h to flip orientation.
   • Repaint strategy: on page change → clear once. Per render
     tick → fillRect over each line's bounding box and redraw it.
     This avoids the full-screen flicker that comes from
     clearScreen() every frame.
   • Module status is colour-coded:
        OK       → green
        DEGRADED → amber
        FAIL     → red
 */

#include "tft_driver.h"

#if HAS_TFT
#include <TFT_eSPI.h>
#include <SPI.h>
#include <string.h>
#include <math.h>

namespace {

TFT_eSPI    screen_ = TFT_eSPI();
TFT_eSprite lineSpr_(&screen_);   // reusable W x LINE_H sprite for body lines
bool        lineSprOk_ = false;
TFT_eSprite homeSpr_(&screen_);   // full-screen sprite for the home page
bool        homeSprOk_ = false;
bool     ready_      = false;
int      lastPage_   = -1;     // forces full clear on first render
bool     forceClear_ = true;   // set when entering a non-page screen
char     lastTitle_[24] = {0}; // last header text — skip redraw if unchanged

// ── layout constants (set at runtime from screen_.width()/height() ──
// so this works for either portrait (128x160) or landscape (160x128).
int W = 128;
int H = 160;
constexpr int HEADER_H = 12;       // slim title + state-badge boxes
constexpr int LINE_H   = 14;       // height of a body text line (FONT 2)
constexpr int BODY_Y0  = HEADER_H + 2;

// 16-bit RGB565 colours.  The display has no backlight-control pin
// (TFT_BL=-1), so panel brightness is fixed at hardware max — we get
// "brighter" UI by avoiding low-luminance shades.  C_DIM is therefore
// repurposed to bright red, which both reads at a glance and matches
// the red-on-black tactical-HUD aesthetic the user requested.
constexpr uint16_t C_BG       = TFT_BLACK;
constexpr uint16_t C_HEADER   = TFT_WHITE;  // header bar background
constexpr uint16_t C_TEXT     = TFT_WHITE;
constexpr uint16_t C_DIM      = TFT_RED;    // ← was 0x8410 grey; now bright red
constexpr uint16_t C_OK       = TFT_GREEN;
constexpr uint16_t C_WARN     = TFT_YELLOW;
constexpr uint16_t C_FAIL     = TFT_RED;
constexpr uint16_t C_ACCENT   = TFT_CYAN;

uint16_t stateColor(ModuleState s) {
  switch (s) {
    case ModuleState::OK:       return C_OK;
    case ModuleState::DEGRADED: return C_WARN;
    default:                    return C_FAIL;
  }
}

const char* stateText(ModuleState s) {
  switch (s) {
    case ModuleState::OK:       return "OK";
    case ModuleState::DEGRADED: return "DEGRADED";
    default:                    return "FAIL";
  }
}

// Draw one body text line.
// Renders into an off-screen sprite, then pushes the whole strip to the
// display in a single SPI burst.  This eliminates the black-then-text
// flicker you'd see with fillRect+drawString directly on the panel.
void line(int idx, const char* text, uint16_t color = C_TEXT) {
  int y = BODY_Y0 + idx * LINE_H;
  if (!lineSprOk_) {
    // Fallback: direct draw (shouldn't happen if init succeeded)
    screen_.fillRect(0, y, W, LINE_H, C_BG);
    screen_.setTextColor(color, C_BG);
    screen_.setTextDatum(TL_DATUM);
    screen_.drawString(text, 2, y, 2);
    return;
  }
  lineSpr_.fillSprite(C_BG);
  lineSpr_.setTextColor(color, C_BG);
  lineSpr_.setTextDatum(TL_DATUM);
  lineSpr_.drawString(text, 2, 0, 2);  // FONT 2 (16px tall classic font)
  lineSpr_.pushSprite(0, y);
}

// printf-style helper around line()
void linef(int idx, uint16_t color, const char* fmt, ...) {
  char buf[40];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  line(idx, buf, color);
}

// Repaint the colored title bar with new text — skipped if title unchanged.
void header(const char* title) {
  if (strncmp(title, lastTitle_, sizeof(lastTitle_)) == 0) return;
  strncpy(lastTitle_, title, sizeof(lastTitle_) - 1);
  lastTitle_[sizeof(lastTitle_) - 1] = '\0';
  screen_.fillRect(0, 0, W, HEADER_H, C_HEADER);
  screen_.setTextColor(C_BG, C_HEADER);   // black text on white header bar
  screen_.setTextDatum(TL_DATUM);
  screen_.drawString(title, 2, 1, 2);
}

void clearAll() {
  screen_.fillScreen(C_BG);
  lastTitle_[0] = '\0';   // force header redraw after a wipe
}

// ─────────────────────────────────────────────────────────────────
//  HOME SCREEN — tactical HUD layout
//
//   ┌──────────────────────────────────────────────┐
//   │ [BT]                                  [GP]  │  status badges
//   │                                              │
//   │              9 4 3   .  5                    │  large range
//   │                METERS                        │
//   │                                              │
//   │   PIT   │   ALT   │   TMP                    │  3-col telemetry
//   │   12°   │  234m   │   24C                    │
//   │ ─────────────────────────────────────────── │  divider
//   │  | N |  NE  |  E  |  SE |  S |               │  compass ticks
//   │                  127°                        │  current heading
//   └──────────────────────────────────────────────┘
// ─────────────────────────────────────────────────────────────────

void drawBleIcon(TFT_eSprite& s, int x, int y, bool connected) {
  // Connected = yellow accent, disconnected = white (was blue).
  uint16_t c = connected ? C_ACCENT : C_TEXT;
  s.drawRoundRect(x, y, 24, 12, 2, c);
  s.setTextDatum(MC_DATUM);
  s.setTextColor(c, C_BG);
  s.drawString("BT", x + 12, y + 6, 1);
}

void drawGpsIcon(TFT_eSprite& s, int x, int y, bool fix) {
  // Connected/fix = yellow accent, no fix = white outline.
  uint16_t c = fix ? C_ACCENT : C_TEXT;
  s.drawRoundRect(x, y, 24, 12, 2, c);
  s.setTextDatum(MC_DATUM);
  s.setTextColor(c, C_BG);
  s.drawString("GP", x + 12, y + 6, 1);
}

void drawCompassStrip(TFT_eSprite& s, float heading, int yTop) {
  const int FOV = 120;                                // visible degrees
  const float pxPerDeg = (float)W / (float)FOV;

  // Normalise heading
  while (heading <    0.0f) heading += 360.0f;
  while (heading >= 360.0f) heading -= 360.0f;

  static const struct { int deg; const char* lbl; } cardinals[] = {
    {  0, "N"  }, { 45, "NE" }, { 90, "E"  }, {135, "SE" },
    {180, "S"  }, {225, "SW" }, {270, "W"  }, {315, "NW" },
  };

  // Tick marks + cardinal labels
  s.setTextDatum(TC_DATUM);
  for (auto& c : cardinals) {
    float delta = (float)c.deg - heading;
    while (delta >  180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    if (fabsf(delta) > FOV / 2.0f) continue;

    int x = W / 2 + (int)(delta * pxPerDeg);
    s.drawFastVLine(x, yTop, 4, C_TEXT);
    s.setTextColor(C_TEXT, C_BG);
    s.drawString(c.lbl, x, yTop + 5, 1);
  }

  // Center indicator triangle (▼) marking current bearing
  int cx = W / 2;
  s.fillTriangle(cx - 3, yTop - 4, cx + 3, yTop - 4, cx, yTop, C_ACCENT);

  // Numeric heading below the strip — no leading zeros, draw a small
  // circle to render the degree symbol (FONT 2 has no ° glyph).
  char buf[8];
  snprintf(buf, sizeof(buf), "%d", (int)(heading + 0.5f));
  s.setTextColor(C_ACCENT, C_BG);
  s.setTextDatum(TC_DATUM);
  s.drawString(buf, cx, yTop + 12, 2);
  // Place the ° circle just to the right of the number.
  int hwid = s.textWidth(buf, 2);
  int dx = cx + hwid / 2 + 4;
  int dy = yTop + 14;
  s.drawCircle(dx, dy, 2, C_ACCENT);
}

void renderHome(const TelemetrySnapshot& snap,
                bool bleConnected,
                int  /*queuedPins*/)
{
  if (!homeSprOk_) {
    // Allocation failed — render minimal placeholder direct-to-panel
    screen_.fillScreen(C_BG);
    screen_.setTextDatum(MC_DATUM);
    screen_.setTextColor(C_ACCENT, C_BG);
    screen_.drawString("HOME: low memory", W / 2, H / 2, 2);
    return;
  }

  // ── 1) wipe sprite ──────────────────────────────────────────
  homeSpr_.fillSprite(C_BG);

  // ── 2) status bar (y 0-12) ─────────────────────────────────
  drawBleIcon    (homeSpr_, 2,        2, bleConnected);
  drawGpsIcon    (homeSpr_, W - 26,   2, snap.gnss.fix >= GnssFix::FIX_2D);

  // ── 3) range readout (FONT 4 ≈ 26 px, comfortably sized) ───
  homeSpr_.setTextDatum(TC_DATUM);
  if (snap.lidar.valid) {
    char rng[10];
    if (snap.lidar.rangeM < 100.0f)
      snprintf(rng, sizeof(rng), "%.1f", snap.lidar.rangeM);
    else
      snprintf(rng, sizeof(rng), "%.0f", snap.lidar.rangeM);
    homeSpr_.setTextColor(C_ACCENT, C_BG);
    homeSpr_.drawString(rng, W / 2, 18, 4);
  } else {
    homeSpr_.setTextColor(C_TEXT, C_BG);
    homeSpr_.drawString("---", W / 2, 18, 4);
  }
  // unit label
  homeSpr_.setTextColor(C_TEXT, C_BG);
  homeSpr_.drawString("METERS", W / 2, 46, 1);

  // ── 4) 3-column telemetry (y 58-80) ────────────────────────
  const int colW = W / 3;
  const int colY_lbl = 58;
  const int colY_val = 68;

  auto drawCol = [&](int col, const char* lbl, const char* val) {
    int cx = col * colW + colW / 2;
    homeSpr_.setTextDatum(TC_DATUM);
    homeSpr_.setTextColor(C_TEXT, C_BG);
    homeSpr_.drawString(lbl, cx, colY_lbl, 1);
    homeSpr_.setTextColor(C_ACCENT, C_BG);
    homeSpr_.drawString(val, cx, colY_val, 2);
  };

  char pitBuf[8], altBuf[10], tmpBuf[10];
  snprintf(pitBuf, sizeof(pitBuf), "%+.0f", snap.imu.pitch);
  if (snap.baro.state == ModuleState::OK || snap.baro.state == ModuleState::DEGRADED)
    snprintf(altBuf, sizeof(altBuf), "%.0fm", snap.baro.altEstM);
  else
    snprintf(altBuf, sizeof(altBuf), "--m");
  if (snap.baro.state == ModuleState::OK || snap.baro.state == ModuleState::DEGRADED)
    snprintf(tmpBuf, sizeof(tmpBuf), "%.0fC", snap.baro.tempC);
  else
    snprintf(tmpBuf, sizeof(tmpBuf), "--C");

  drawCol(0, "PIT", pitBuf);
  drawCol(1, "ALT", altBuf);
  drawCol(2, "TMP", tmpBuf);

  // column separators
  homeSpr_.drawFastVLine(colW,     colY_lbl, 22, C_TEXT);
  homeSpr_.drawFastVLine(colW * 2, colY_lbl, 22, C_TEXT);

  // ── 5) divider line ───────────────────────────────────────────────
  homeSpr_.drawFastHLine(0, 84, W, C_TEXT);

  // ── 6) compass strip (y 88-124) ──────────────────────────────
  float hdg = (snap.imu.state == ModuleState::FAIL) ? 0.0f : snap.imu.heading;
  drawCompassStrip(homeSpr_, hdg, 92);

  // If IMU is dead, dim-tag near the heading number
  if (snap.imu.state == ModuleState::FAIL) {
    homeSpr_.setTextDatum(TC_DATUM);
    homeSpr_.setTextColor(C_DIM, C_BG);
    homeSpr_.drawString("IMU", 12, 118, 1);
  }

  // ── 7) push the whole frame in a single SPI burst ──────────
  homeSpr_.pushSprite(0, 0);

  // ── 8) belt-and-braces edge cleanup ──────────────────────
  // Some ST7735 1.8" panels have a multi-pixel offset that not every
  // tab variant accounts for, leaving uninitialised GRAM at the very
  // edges.  Paint the last 8 rows + last 2 columns black directly on
  // the panel so any visible offset rows show as background.
  screen_.fillRect(0, H - 8, W, 8, C_BG);
  screen_.fillRect(W - 2, 0, 2, H, C_BG);
}

// ─────────────────────────────────────────────────────────────────
//  POLISHED DETAIL PAGES — same look-and-feel as the home page,
//  composed entirely off-screen and pushed in one SPI burst.
// ─────────────────────────────────────────────────────────────────

// Top-row UI: two outlined rounded boxes — left holds the sensor name in
// white, right holds the OK/DEGRADED/FAIL badge in green/yellow/red.  No
// solid fills, matches the home-page BT/GP icon style.
void drawDetailHeader(const char* title, ModuleState st) {
  const int titleW = 48;
  const int badgeW = 52;
  // Title box (white outline, white text)
  homeSpr_.drawRoundRect(0, 0, titleW, HEADER_H, 2, C_HEADER);
  homeSpr_.setTextDatum(MC_DATUM);
  homeSpr_.setTextColor(C_HEADER, C_BG);
  homeSpr_.drawString(title, titleW / 2, HEADER_H / 2, 1);
  // State badge (state-color outline + text), inset 3 px from the right
  // edge so the per-frame edge mask doesn't clip/flicker the border.
  uint16_t sc = stateColor(st);
  int bx = W - badgeW - 3;
  homeSpr_.drawRoundRect(bx, 0, badgeW, HEADER_H, 2, sc);
  homeSpr_.setTextColor(sc, C_BG);
  homeSpr_.drawString(stateText(st), bx + badgeW / 2, HEADER_H / 2, 1);
}

// One big number centered with a small unit beneath.  Default FONT 4
// (~26 px tall) keeps headroom for a 2-col stat grid below.
void drawHero(const char* val, const char* unit, int yTop, uint8_t font = 4) {
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.setTextColor(C_ACCENT, C_BG);
  homeSpr_.drawString(val, W / 2, yTop, font);
  homeSpr_.setTextColor(C_TEXT, C_BG);
  int unitDy = (font == 6) ? 30 : (font == 4) ? 22 : 16;
  homeSpr_.drawString(unit, W / 2, yTop + unitDy, 1);
}

// Big number with degree-symbol circle drawn to the right of the digits.
// Used for IMU heading.
void drawHeroDeg(int deg, const char* unit, int yTop, uint8_t font = 4) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d", deg);
  int cx = W / 2;
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.setTextColor(C_ACCENT, C_BG);
  homeSpr_.drawString(buf, cx, yTop, font);
  // ° circle
  int hwid = homeSpr_.textWidth(buf, font);
  int r = (font >= 6) ? 3 : 2;
  int dx = cx + hwid / 2 + r + 2;
  int dy = yTop + r + 2;
  homeSpr_.drawCircle(dx, dy, r, C_ACCENT);
  // unit label below
  homeSpr_.setTextColor(C_TEXT, C_BG);
  int unitDy = (font == 6) ? 30 : (font == 4) ? 22 : 16;
  homeSpr_.drawString(unit, cx, yTop + unitDy, 1);
}

// 2-column stat grid: label small (white) over value (yellow, FONT 2).
void drawStat2Col(const char* lblL, const char* valL,
                  const char* lblR, const char* valR, int yTop) {
  int qW = W / 4;
  int cxL = qW;
  int cxR = qW * 3;
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.setTextColor(C_TEXT, C_BG);
  homeSpr_.drawString(lblL, cxL, yTop, 1);
  homeSpr_.drawString(lblR, cxR, yTop, 1);
  homeSpr_.setTextColor(C_ACCENT, C_BG);
  homeSpr_.drawString(valL, cxL, yTop + 9, 2);
  homeSpr_.drawString(valR, cxR, yTop + 9, 2);
  homeSpr_.drawFastVLine(W / 2, yTop, 24, C_TEXT);
}

void pushDetailFrame() {
  homeSpr_.pushSprite(0, 0);
  // Edge cleanup matches home page.
  screen_.fillRect(0, H - 8, W, 8, C_BG);
  screen_.fillRect(W - 2, 0, 2, H, C_BG);
}

void renderImuPage(const TelemetrySnapshot& snap) {
  if (!homeSprOk_) return;
  homeSpr_.fillSprite(C_BG);
  drawDetailHeader("IMU", snap.imu.state);
  // Big heading number with ° symbol
  int hdg = (snap.imu.state == ModuleState::FAIL) ? 0 : (int)(snap.imu.heading + 0.5f);
  drawHeroDeg(hdg, "HEADING", HEADER_H + 6, 4);
  // Pitch / Roll 2-col grid
  char pBuf[10], rBuf[10];
  snprintf(pBuf, sizeof(pBuf), "%+.0f", snap.imu.pitch);
  snprintf(rBuf, sizeof(rBuf), "%+.0f", snap.imu.roll);
  drawStat2Col("PITCH", pBuf, "ROLL", rBuf, H - 38);
  pushDetailFrame();
}

void renderGnssPage(const TelemetrySnapshot& snap) {
  if (!homeSprOk_) return;
  homeSpr_.fillSprite(C_BG);
  drawDetailHeader("GNSS", snap.gnss.state);

  // Hero: fix label (smaller — FONT 4 not 6)
  const char* fixStr =
    snap.gnss.fix == GnssFix::FIX_3D ? "3D" :
    snap.gnss.fix == GnssFix::FIX_2D ? "2D" : "--";
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.setTextColor(snap.gnss.fix >= GnssFix::FIX_2D ? C_ACCENT : C_TEXT, C_BG);
  homeSpr_.drawString(fixStr, W / 2, HEADER_H + 4, 4);
  homeSpr_.setTextColor(C_TEXT, C_BG);
  homeSpr_.drawString("FIX", W / 2, HEADER_H + 26, 1);

  // Lat / Lon stacked, FONT 1 to fit comfortably
  char latBuf[20], lonBuf[20];
  if (snap.gnss.fix >= GnssFix::FIX_2D) {
    snprintf(latBuf, sizeof(latBuf), "LAT %.5f", snap.gnss.lat);
    snprintf(lonBuf, sizeof(lonBuf), "LON %.5f", snap.gnss.lon);
  } else {
    snprintf(latBuf, sizeof(latBuf), "LAT ---");
    snprintf(lonBuf, sizeof(lonBuf), "LON ---");
  }
  homeSpr_.setTextColor(C_ACCENT, C_BG);
  homeSpr_.drawString(latBuf, W / 2, HEADER_H + 42, 2);
  homeSpr_.drawString(lonBuf, W / 2, HEADER_H + 58, 2);

  // Sats / Acc 2-col at bottom
  char sBuf[8], aBuf[10];
  snprintf(sBuf, sizeof(sBuf), "%d", snap.gnss.sats);
  snprintf(aBuf, sizeof(aBuf), "%.1fm", snap.gnss.accM);
  drawStat2Col("SATS", sBuf, "ACC", aBuf, H - 38);
  pushDetailFrame();
}

void renderBaroPage(const TelemetrySnapshot& snap) {
  if (!homeSprOk_) return;
  homeSpr_.fillSprite(C_BG);
  drawDetailHeader("BARO", snap.baro.state);

  // Hero: altitude (FONT 4 — smaller than before)
  char altBuf[10];
  bool ok = snap.baro.state != ModuleState::FAIL;
  if (ok) snprintf(altBuf, sizeof(altBuf), "%.0f", snap.baro.altEstM);
  else    snprintf(altBuf, sizeof(altBuf), "--");
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.setTextColor(ok ? C_ACCENT : C_TEXT, C_BG);
  homeSpr_.drawString(altBuf, W / 2, HEADER_H + 4, 4);
  homeSpr_.setTextColor(C_TEXT, C_BG);
  homeSpr_.drawString("METERS ASL", W / 2, HEADER_H + 26, 1);

  // Pressure / Temp 2-col
  char pBuf[12], tBuf[10];
  if (ok) {
    snprintf(pBuf, sizeof(pBuf), "%.1f", snap.baro.pressHPa);
    snprintf(tBuf, sizeof(tBuf), "%.1fC", snap.baro.tempC);
  } else {
    snprintf(pBuf, sizeof(pBuf), "--");
    snprintf(tBuf, sizeof(tBuf), "--");
  }
  drawStat2Col("hPa", pBuf, "TEMP", tBuf, H - 38);
  pushDetailFrame();
}

void renderLidarPage(const TelemetrySnapshot& snap) {
  if (!homeSprOk_) return;
  homeSpr_.fillSprite(C_BG);
  drawDetailHeader("LIDAR", snap.lidar.state);

  // Hero: range (FONT 4)
  char rngBuf[10];
  bool ok = snap.lidar.valid;
  if (ok) {
    if (snap.lidar.rangeM < 100.0f)
      snprintf(rngBuf, sizeof(rngBuf), "%.1f", snap.lidar.rangeM);
    else
      snprintf(rngBuf, sizeof(rngBuf), "%.0f", snap.lidar.rangeM);
  } else {
    snprintf(rngBuf, sizeof(rngBuf), "--");
  }
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.setTextColor(ok ? C_ACCENT : C_TEXT, C_BG);
  homeSpr_.drawString(rngBuf, W / 2, HEADER_H + 4, 4);
  homeSpr_.setTextColor(C_TEXT, C_BG);
  homeSpr_.drawString("METERS", W / 2, HEADER_H + 26, 1);

  // Quality / Valid 2-col
  char qBuf[8];
  snprintf(qBuf, sizeof(qBuf), "%d", snap.lidar.quality);
  drawStat2Col("QUAL", qBuf, "VALID", ok ? "YES" : "NO", H - 38);
  pushDetailFrame();
}

// Modern at-a-glance sensor health page \u2014 lists each subsystem with a
// colored OK / DEGRADED / FAIL pill on the right.  Replaces the legacy
// serial-style overview.
void renderSensorsPage(const TelemetrySnapshot& snap, bool bleConnected) {
  if (!homeSprOk_) return;
  homeSpr_.fillSprite(C_BG);

  // Title-only header (no overall-state badge — the per-row pills below
  // already convey full status, the redundant top-right badge was noisy).
  const int titleW = 70;
  homeSpr_.drawRoundRect(0, 0, titleW, HEADER_H, 2, C_HEADER);
  homeSpr_.setTextDatum(MC_DATUM);
  homeSpr_.setTextColor(C_HEADER, C_BG);
  homeSpr_.drawString("SENSORS", titleW / 2, HEADER_H / 2, 1);

  // Five rows: IMU / GNSS / BARO / LIDAR / BLE.  BLE has its own vocabulary
  // (CONN / DISCONN) instead of the generic OK / DEGRADED / FAIL.
  struct Row { const char* name; ModuleState st; const char* label; };
  Row rows[5] = {
    { "IMU",   snap.imu.state,   nullptr },
    { "GNSS",  snap.gnss.state,  nullptr },
    { "BARO",  snap.baro.state,  nullptr },
    { "LIDAR", snap.lidar.state, nullptr },
    { "BLE",
      bleConnected ? ModuleState::OK : ModuleState::DEGRADED,
      bleConnected ? "CONN"          : "DISCONN" },
  };

  const int rowH    = 20;
  const int yStart  = HEADER_H + 4;
  const int badgeW  = 70;
  const int badgeH  = 16;
  const int badgeX  = W - badgeW - 4;

  for (int i = 0; i < 5; ++i) {
    int y = yStart + i * rowH;
    homeSpr_.setTextDatum(ML_DATUM);
    homeSpr_.setTextColor(C_TEXT, C_BG);
    homeSpr_.drawString(rows[i].name, 6, y + badgeH / 2, 2);
    uint16_t sc = stateColor(rows[i].st);
    homeSpr_.drawRoundRect(badgeX, y, badgeW, badgeH, 3, sc);
    homeSpr_.setTextDatum(MC_DATUM);
    homeSpr_.setTextColor(sc, C_BG);
    homeSpr_.drawString(rows[i].label ? rows[i].label : stateText(rows[i].st),
                        badgeX + badgeW / 2, y + badgeH / 2, 1);
  }

  pushDetailFrame();
}

}  // namespace

namespace tft {

bool init() {
  screen_.init();
  screen_.setRotation(cfg::TFT_ROTATION);

  // Reflect the complete framebuffer left-to-right for the HUD combiner.
  // setRotation() is called first so TFT_eSPI still configures the correct
  // dimensions and ST7735 panel offsets; only the address direction changes.
  if (cfg::TFT_MIRROR_HORIZONTAL) {
    uint8_t madctl;
    switch (cfg::TFT_ROTATION & 3) {
      case 0:  madctl = TFT_MAD_MY;                         break;
      case 1:  madctl = TFT_MAD_MV;                         break;
      case 2:  madctl = TFT_MAD_MX;                         break;
      default: madctl = TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_MV; break;
    }
    screen_.writecommand(TFT_MADCTL);
    screen_.writedata(madctl | TFT_MAD_COLOR_ORDER);
  }

  W = screen_.width();
  H = screen_.height();
  screen_.fillScreen(C_BG);
  screen_.setTextColor(C_TEXT, C_BG);
  screen_.setTextDatum(TL_DATUM);

  // Allocate the per-line off-screen buffer used by line()/linef() so
  // body text updates are flicker-free single-burst pushes.
  lineSpr_.setColorDepth(16);
  if (lineSpr_.createSprite(W, LINE_H) != nullptr) {
    lineSprOk_ = true;
    lineSpr_.fillSprite(C_BG);
  } else {
    lineSprOk_ = false;   // fall back to direct draw
  }

  // Allocate full-screen sprite for the home page (~40 KB at 160x128x16bpp).
  // The home page repaints a lot of dynamic content each tick — doing it
  // off-screen and pushing once eliminates all flicker.
  homeSpr_.setColorDepth(16);
  if (homeSpr_.createSprite(W, H) != nullptr) {
    homeSprOk_ = true;
    homeSpr_.fillSprite(C_BG);
  } else {
    homeSprOk_ = false;
  }

  ready_      = true;
  lastPage_   = -1;
  forceClear_ = true;
  lastTitle_[0] = '\0';
  return ready_;
}

bool isReady() { return ready_; }

void renderBootSplash() {
  if (!ready_) return;
  clearAll();
  if (!homeSprOk_) {
    // Fallback: simple text splash if sprite alloc failed.
    screen_.setTextColor(C_ACCENT, C_BG);
    screen_.setTextDatum(MC_DATUM);
    screen_.drawString("L.I.N.K.", W / 2, H / 2 - 14, 4);
    screen_.setTextColor(C_DIM, C_BG);
    screen_.drawString("Booting...", W / 2, H / 2 + 12, 2);
    forceClear_ = true;
    return;
  }

  homeSpr_.fillSprite(C_BG);

  // Decorative top/bottom hairlines in cyan-rendered-yellow accent.
  homeSpr_.drawFastHLine(8, 8,    W - 16, C_ACCENT);
  homeSpr_.drawFastHLine(8, H - 9, W - 16, C_ACCENT);

  // Big title block.
  homeSpr_.setTextDatum(MC_DATUM);
  homeSpr_.setTextColor(C_ACCENT, C_BG);
  homeSpr_.drawString("L.I.N.K.", W / 2, H / 2 - 10, 4);

  // "Booting" label with three animated-ish trailing dots that stay static
  // (no timer state on this static splash) but read as a progress hint.
  homeSpr_.setTextColor(C_OK, C_BG);
  homeSpr_.drawString("booting...", W / 2, H / 2 + 18, 2);

  // Indeterminate progress bar across the lower third.
  const int pbY = H - 22;
  const int pbX = 16;
  const int pbW = W - 32;
  const int pbH = 6;
  homeSpr_.drawRoundRect(pbX, pbY, pbW, pbH, 2, C_DIM);
  // Filled segment in green covering ~40 % — a static "working on it" hint.
  homeSpr_.fillRoundRect(pbX + 2, pbY + 2, (pbW - 4) * 4 / 10, pbH - 4, 1, C_OK);

  homeSpr_.pushSprite(0, 0);
  screen_.fillRect(0, H - 2, W, 2, C_BG);
  screen_.fillRect(W - 2, 0, 2, H, C_BG);
  forceClear_ = true;
}

void renderReadyScreen(bool gnssOk) {
  if (!ready_) return;
  clearAll();
  screen_.setTextDatum(MC_DATUM);
  screen_.setTextColor(C_OK, C_BG);
  screen_.drawString("L.I.N.K.", W / 2, H * 1 / 5, 4);
  screen_.setTextColor(C_TEXT, C_BG);
  screen_.drawString("Ready", W / 2, H * 2 / 5, 2);
  screen_.setTextColor(gnssOk ? C_OK : C_WARN, C_BG);
  screen_.drawString(gnssOk ? "GPS: OK" : "GPS: --", W / 2, H * 3 / 5, 2);
  screen_.setTextColor(C_DIM, C_BG);
  screen_.drawString("Waiting for BLE...", W / 2, H * 4 / 5, 2);
  forceClear_ = true;
}

void renderPingFail(const char* line2) {
  if (!ready_) return;
  clearAll();
  screen_.setTextDatum(MC_DATUM);
  screen_.setTextColor(C_FAIL, C_BG);
  screen_.drawString("PING FAIL", W / 2, H / 2 - 14, 4);
  screen_.setTextColor(C_TEXT, C_BG);
  screen_.drawString(line2 ? line2 : "", W / 2, H / 2 + 12, 2);
  forceClear_ = true;
}

void renderPingHold(uint8_t percent) {
  if (!ready_) return;
  if (lastPage_ != -2 || forceClear_) {
    lastPage_   = -2;
    forceClear_ = false;
  }

  if (!homeSprOk_) {
    // Fallback path — flickery but safe.
    screen_.fillScreen(C_BG);
    screen_.setTextColor(C_WARN, C_BG);
    screen_.setTextDatum(TC_DATUM);
    screen_.drawString("DROPPING PIN", W / 2, 6, 2);
    return;
  }

  homeSpr_.fillSprite(C_BG);

  // Title — small caption at the very top, no header bar.
  homeSpr_.setTextColor(C_WARN, C_BG);
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.drawString("DROPPING PIN", W / 2, 2, 2);

  // Anti-aliased ring centered horizontally, slightly above middle so the
  // two caption lines below fit cleanly above the bottom edge mask.
  const int cx = W / 2;
  const int cy = 60;
  const int rOuter = 26;
  const int rInner = 19;

  // Background track — full circle, dim red (renders as muted blue on this
  // BGR panel) so the green foreground pops.
  homeSpr_.drawSmoothArc(cx, cy, rOuter, rInner, 0, 360,
                         C_DIM, C_BG, true);

  // Foreground arc — TFT_eSPI angles: 0° = 6 o'clock, increases clockwise.
  // Start at top (180°) and sweep clockwise; visualise wrap-around manually.
  if (percent > 0) {
    int sweep = (int)percent * 360 / 100;
    int start = 180;
    int end   = 180 + sweep;
    if (end <= 360) {
      homeSpr_.drawSmoothArc(cx, cy, rOuter, rInner, start, end,
                             C_OK, C_BG, true);
    } else {
      homeSpr_.drawSmoothArc(cx, cy, rOuter, rInner, 180, 360,
                             C_OK, C_BG, true);
      homeSpr_.drawSmoothArc(cx, cy, rOuter, rInner, 0, end - 360,
                             C_OK, C_BG, true);
    }
  }

  // Percentage centered — FONT 2 keeps the glyphs inside the inner radius
  // (FONT 4 was bleeding into the ring at 100%).
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", percent);
  homeSpr_.setTextDatum(MC_DATUM);
  homeSpr_.setTextColor(C_ACCENT, C_BG);
  homeSpr_.drawString(buf, cx, cy, 2);

  // Caption under ring — small font, both lines guaranteed to fit above
  // the H-8..H-1 edge mask.
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.setTextColor(C_TEXT, C_BG);
  homeSpr_.drawString("HOLD TO CONFIRM", W / 2, cy + rOuter + 6, 1);
  homeSpr_.setTextColor(C_DIM, C_BG);
  homeSpr_.drawString("release to cancel", W / 2, cy + rOuter + 18, 1);

  homeSpr_.pushSprite(0, 0);
  screen_.fillRect(0, H - 8, W, 8, C_BG);
  screen_.fillRect(W - 2, 0, 2, H, C_BG);
}

void renderPingAbort(const char* reason) {
  if (!ready_) return;
  if (lastPage_ != -3 || forceClear_) {
    lastPage_   = -3;
    forceClear_ = false;
  }
  if (!homeSprOk_) {
    screen_.fillScreen(C_BG);
    screen_.setTextColor(C_FAIL, C_BG);
    screen_.setTextDatum(TC_DATUM);
    screen_.drawString("PING ABORTED", W / 2, 8, 2);
    return;
  }
  homeSpr_.fillSprite(C_BG);

  // Red outlined alert box top
  homeSpr_.drawRoundRect(4, 4, W - 8, 22, 3, C_FAIL);
  homeSpr_.setTextColor(C_FAIL, C_BG);
  homeSpr_.setTextDatum(MC_DATUM);
  homeSpr_.drawString("PING ABORTED", W / 2, 15, 2);

  // Big sensor name in red
  homeSpr_.setTextColor(C_FAIL, C_BG);
  homeSpr_.drawString(reason ? reason : "?", W / 2, H / 2 - 4, 4);

  // Subtitle
  homeSpr_.setTextColor(C_TEXT, C_BG);
  homeSpr_.drawString("sensor unavailable", W / 2, H / 2 + 22, 2);
  homeSpr_.setTextColor(C_DIM, C_BG);
  homeSpr_.drawString("retry when ready", W / 2, H - 16, 1);

  homeSpr_.pushSprite(0, 0);
  screen_.fillRect(0, H - 8, W, 8, C_BG);
  screen_.fillRect(W - 2, 0, 2, H, C_BG);
}

// Brief "PING SENT" confirmation \u2014 large green check, total queued count.
void renderPingSent(int queuedPins) {
  if (!ready_) return;
  if (lastPage_ != -4 || forceClear_) {
    lastPage_   = -4;
    forceClear_ = false;
  }
  if (!homeSprOk_) {
    screen_.fillScreen(C_BG);
    screen_.setTextColor(C_OK, C_BG);
    screen_.setTextDatum(TC_DATUM);
    screen_.drawString("PING SENT", W / 2, 8, 2);
    return;
  }
  homeSpr_.fillSprite(C_BG);

  // Green outlined banner
  homeSpr_.drawRoundRect(4, 4, W - 8, 22, 3, C_OK);
  homeSpr_.setTextColor(C_OK, C_BG);
  homeSpr_.setTextDatum(MC_DATUM);
  homeSpr_.drawString("PING SENT", W / 2, 15, 2);

  // Big check mark drawn from two thick lines.
  const int cx = W / 2;
  const int cy = H / 2 + 2;
  const int s  = 16;
  for (int t = -1; t <= 1; ++t) {
    homeSpr_.drawLine(cx - s,     cy + t,         cx - 4, cy + s - 4 + t, C_OK);
    homeSpr_.drawLine(cx - 4 + t, cy + s - 4,     cx + s, cy - s + t,     C_OK);
  }

  // Footer — both lines kept above the H-8..H-1 bottom edge mask so they
  // aren't clipped (the dim color rendering as blue made the clipping
  // visually obvious).
  char qbuf[24];
  snprintf(qbuf, sizeof(qbuf), "queued: %d", queuedPins);
  homeSpr_.setTextDatum(TC_DATUM);
  homeSpr_.setTextColor(C_TEXT, C_BG);
  homeSpr_.drawString(qbuf, W / 2, H - 36, 2);
  homeSpr_.setTextColor(C_DIM, C_BG);
  homeSpr_.drawString("syncing to phone...", W / 2, H - 18, 1);

  homeSpr_.pushSprite(0, 0);
  screen_.fillRect(0, H - 8, W, 8, C_BG);
  screen_.fillRect(W - 2, 0, 2, H, C_BG);
}

void renderPage(int hudPage,
                int totalPages,
                const TelemetrySnapshot& snap,
                bool bleConnected,
                int  queuedPins)
{
  if (!ready_) return;

  // Page change → wipe screen once
  if (hudPage != lastPage_ || forceClear_) {
    clearAll();
    lastPage_   = hudPage;
    forceClear_ = false;
  }

  // ── HOME page (0) gets its own custom layout — no header bar ──
  if (hudPage == 0) {
    renderHome(snap, bleConnected, queuedPins);
    return;
  }

  // ── Polished detail pages (1-5): sprite-composed, no flicker ──
  switch (hudPage) {
    case 1: renderSensorsPage(snap, bleConnected); return;
    case 2: renderImuPage(snap);   return;
    case 3: renderGnssPage(snap);  return;
    case 4: renderBaroPage(snap);  return;
    case 5: renderLidarPage(snap); return;
    default: break;
  }

  // Fallback for any unmapped page index.
  static const char* PAGE_NAMES[] = { "HOME", "SENSORS", "IMU", "GNSS", "BARO", "LiDAR" };
  char title[24];
  snprintf(title, sizeof(title), "[%d/%d] %s",
           hudPage + 1, totalPages,
           (hudPage >= 0 && hudPage < (int)(sizeof(PAGE_NAMES)/sizeof(*PAGE_NAMES)))
             ? PAGE_NAMES[hudPage] : "");
  header(title);
  line(0, "Unknown page", C_FAIL);
  (void)queuedPins;
}

}  // namespace tft

#endif  // HAS_TFT

