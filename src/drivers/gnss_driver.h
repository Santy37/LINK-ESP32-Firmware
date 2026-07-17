#pragma once
#include "sensor_types.h"

struct GnssDiag {
  uint8_t  satsInView;     // sats the receiver can hear (from $GxGSV)
  uint8_t  satsUsed;       // sats actually used for the current fix
  uint8_t  maxSnrDbHz;     // strongest sat right now — antenna health proxy
  uint8_t  avgSnrDbHz;     // mean SNR across sats in view
  uint16_t failedChecksums;// bad NMEA sentences since boot (RF noise / wiring)
  uint32_t goodSentences;  // good NMEA sentences since boot
  float    hdop;           // horizontal dilution of precision
};

bool     gnss_init();
bool     gnss_selfTest();
GnssData gnss_read();
GnssDiag gnss_getDiag();           // antenna-health snapshot
void     gnss_logDiag();           // print a one-line diagnostic to Serial
unsigned long gps_charsProcessed();
