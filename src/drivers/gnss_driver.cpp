/* GNSS Driver — u-blox NEO-M8N / NEO-6M via TinyGPS++
 
 Reads NMEA over UART2, exposes lat/lon/alt/accuracy/fix.
 */

#include "gnss_driver.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "config.h"

static HardwareSerial gpsSerial(2);   // UART2
static TinyGPSPlus    gps;
static bool _initialised = false;

/* GSV-sentence scanner — extracts per-satellite C/N0 (SNR, dB-Hz) so we can
   report antenna health independently of whether we have a fix yet.
   GSV format (variable, up to 4 sats per sentence):
     $GxGSV,total_msgs,msg_num,sats_in_view,prn,elev,azim,snr,...*HH
   We accumulate across all GSV messages of a cycle and snapshot on the
   last-message-of-cycle marker (msg_num == total_msgs).               */
static char     _lineBuf[100];
static uint8_t  _lineLen   = 0;

// Per-constellation last-completed cycle snapshot (GP, GL, GA, GB).
// Aggregate diag = sum across all valid slots.
struct ConstDiag {
  uint8_t  view;
  uint8_t  snrMax;
  uint16_t snrSum;
  uint8_t  snrCnt;
  bool     valid;
};
static ConstDiag _perTalker[4] = {};

// In-progress accumulators for the current cycle (single talker at a time —
// u-blox emits all msgs of one constellation contiguously).
static int8_t   _curTalker  = -1;
static uint8_t  _viewAcc    = 0;
static uint8_t  _snrMaxAcc  = 0;
static uint16_t _snrSumAcc  = 0;
static uint8_t  _snrCntAcc  = 0;

// Aggregate snapshot (read by getDiag)
static uint8_t  _satsInView = 0;
static uint8_t  _snrMax     = 0;
static uint8_t  _snrAvg     = 0;

static int8_t _talkerIndex(char t1, char t2) {
  if (t1 != 'G') return -1;
  switch (t2) {
    case 'P': return 0;  // GPS
    case 'L': return 1;  // GLONASS
    case 'A': return 2;  // Galileo
    case 'B': return 3;  // BeiDou
    default:  return -1;
  }
}

static void _recomputeAggregate() {
  uint16_t view = 0, snrSum = 0;
  uint8_t  snrMax = 0, snrCnt = 0;
  for (int i = 0; i < 4; ++i) {
    if (!_perTalker[i].valid) continue;
    view   += _perTalker[i].view;
    snrSum += _perTalker[i].snrSum;
    snrCnt += _perTalker[i].snrCnt;
    if (_perTalker[i].snrMax > snrMax) snrMax = _perTalker[i].snrMax;
  }
  _satsInView = view > 255 ? 255 : (uint8_t)view;
  _snrMax     = snrMax;
  _snrAvg     = snrCnt > 0 ? (uint8_t)(snrSum / snrCnt) : 0;
}

static void _resetInProgress(int8_t talker) {
  _curTalker  = talker;
  _viewAcc    = 0;
  _snrMaxAcc  = 0;
  _snrSumAcc  = 0;
  _snrCntAcc  = 0;
}

static void _processGsvLine(const char *line, uint8_t len) {
  // Expect line like: $GPGSV,3,2,11,07,..,..,32,08,..,..,28,...*HH
  // field 0: $GxGSV   1: total  2: num  3: sats_in_view
  // then groups of 4 (prn, elev, azim, snr) up to end (snr may be empty)
  int8_t talker = _talkerIndex(line[1], line[2]);
  if (talker < 0) return;

  const char *p = line;
  uint8_t field = 0;
  const char *fStart = p;
  uint8_t totalMsgs = 0, msgNum = 0;
  uint8_t inView = 0;

  while (p <= line + len) {
    if (*p == ',' || *p == '*' || *p == 0 || p == line + len) {
      uint8_t flen = (uint8_t)(p - fStart);
      char tmp[8] = {0};
      uint8_t cp = flen < 7 ? flen : 7;
      memcpy(tmp, fStart, cp);
      tmp[cp] = 0;

      if (field == 1) totalMsgs = (uint8_t)atoi(tmp);
      else if (field == 2) msgNum = (uint8_t)atoi(tmp);
      else if (field == 3) inView = (uint8_t)atoi(tmp);
      else if (field >= 4) {
        uint8_t blockField = (field - 4) % 4;
        if (blockField == 3 && flen > 0) {
          uint8_t snr = (uint8_t)atoi(tmp);
          if (snr > 0) {
            if (snr > _snrMaxAcc) _snrMaxAcc = snr;
            _snrSumAcc += snr;
            _snrCntAcc += 1;
          }
        }
      }

      ++field;
      fStart = p + 1;
      if (*p == '*' || *p == 0) break;
    }
    ++p;
  }

  // Start a fresh in-progress cycle when talker changes or msg #1 arrives
  if (talker != _curTalker || msgNum == 1) _resetInProgress(talker);

  _viewAcc = inView;

  // Commit this constellation's slot at end of its cycle, then recompute
  if (totalMsgs > 0 && msgNum == totalMsgs) {
    _perTalker[talker].view   = _viewAcc;
    _perTalker[talker].snrMax = _snrMaxAcc;
    _perTalker[talker].snrSum = _snrSumAcc;
    _perTalker[talker].snrCnt = _snrCntAcc;
    _perTalker[talker].valid  = true;
    _recomputeAggregate();
    _resetInProgress(-1);
  }
}

static void _feedGsvScanner(char c) {
  if (c == '\n' || c == '\r') {
    if (_lineLen > 6 &&
        _lineBuf[0] == '$' &&
        _lineBuf[3] == 'G' && _lineBuf[4] == 'S' && _lineBuf[5] == 'V') {
      _lineBuf[_lineLen] = 0;
      _processGsvLine(_lineBuf, _lineLen);
    }
    _lineLen = 0;
    return;
  }
  if (_lineLen < sizeof(_lineBuf) - 1) {
    _lineBuf[_lineLen++] = c;
  } else {
    _lineLen = 0;   // overflow → drop line
  }
}

bool gnss_init() {
  // Simple init — matches the working test sketch exactly
  gpsSerial.begin(cfg::GNSS_BAUD, SERIAL_8N1, cfg::GNSS_RX, cfg::GNSS_TX);

  Serial.printf("[GNSS] UART2 started: baud=%ld RX=%d TX=%d\n",
                cfg::GNSS_BAUD, cfg::GNSS_RX, cfg::GNSS_TX);
  Serial.print("[GNSS] Waiting 3s for NMEA data: ");

  // Read raw for 3 seconds — print first 120 chars so we can see what arrives
  int rawCount = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      gps.encode(c);
      _feedGsvScanner(c);
      if (rawCount < 120) Serial.print(c);
      rawCount++;
    }
  }

  Serial.printf("\n[GNSS] raw chars received: %d  TinyGPS processed: %lu\n",
                rawCount, gps.charsProcessed());

  _initialised = (gps.charsProcessed() > 10);
  Serial.printf("[GNSS] init → %s\n", _initialised ? "OK" : "FAIL (no NMEA)");
  return _initialised;
}

bool gnss_selfTest() {
  if (!_initialised) return false;
  // Consider passing if we've decoded at least one sentence
  bool ok = (gps.sentencesWithFix() > 0 || gps.satellites.value() > 0);
  Serial.printf("[GNSS] self-test sats=%d fix=%s → %s\n",
                (int)gps.satellites.value(),
                gps.location.isValid() ? "valid" : "none",
                ok ? "PASS" : "NO FIX YET");
  return ok;
}

GnssData gnss_read() {
  // Drain available bytes
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
    _feedGsvScanner(c);
  }

  GnssData d{};

  if (!_initialised) {
    d.state = ModuleState::FAIL;
    return d;
  }

  // TinyGPS++ keeps the last valid position indefinitely. Require a recent
  // update so a cached location from before signal loss cannot be pinged.
  bool locationFresh = gps.location.isValid() &&
                       gps.location.age() <= cfg::GNSS_MAX_LOCATION_AGE_MS;

  if (locationFresh) {
    d.lat  = gps.location.lat();
    d.lon  = gps.location.lng();
    d.altM = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0;
    d.sats = (uint8_t)gps.satellites.value();

    // Rough accuracy from HDOP (metres ≈ HDOP × 5)
    d.accM = gps.hdop.isValid() ? (float)(gps.hdop.hdop() * 5.0) : 99.0f;

    // Determine fix quality
    if (d.sats >= 6)      d.fix = GnssFix::FIX_3D;
    else if (d.sats >= 3) d.fix = GnssFix::FIX_2D;
    else                   d.fix = GnssFix::NONE;

    d.state = (d.fix >= GnssFix::FIX_2D) ? ModuleState::OK : ModuleState::DEGRADED;
  } else {
    d.fix   = GnssFix::NONE;
    d.state = ModuleState::DEGRADED;
  }

  return d;
}

unsigned long gps_charsProcessed() {
  return gps.charsProcessed();
}

GnssDiag gnss_getDiag() {
  GnssDiag d{};
  d.satsInView      = _satsInView;
  d.satsUsed        = (uint8_t)gps.satellites.value();
  d.maxSnrDbHz      = _snrMax;
  d.avgSnrDbHz      = _snrAvg;
  d.failedChecksums = (uint16_t)gps.failedChecksum();
  d.goodSentences   = (uint32_t)gps.passedChecksum();
  d.hdop            = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 99.9f;
  return d;
}

void gnss_logDiag() {
  GnssDiag d = gnss_getDiag();
  const char *antenna;
  if      (d.maxSnrDbHz == 0) antenna = "NO SIGNAL";
  else if (d.maxSnrDbHz < 25) antenna = "WEAK";
  else if (d.maxSnrDbHz < 35) antenna = "OK";
  else if (d.maxSnrDbHz < 42) antenna = "GOOD";
  else                        antenna = "EXCELLENT";

  Serial.printf("[GNSS] view=%u used=%u maxSNR=%udB avgSNR=%udB hdop=%.1f "
                "ok=%lu bad=%u chars=%lu  antenna=%s\n",
                d.satsInView, d.satsUsed, d.maxSnrDbHz, d.avgSnrDbHz, d.hdop,
                (unsigned long)d.goodSentences, d.failedChecksums,
                gps.charsProcessed(), antenna);
}
