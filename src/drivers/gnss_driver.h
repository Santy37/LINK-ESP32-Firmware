#pragma once
#include "sensor_types.h"

bool     gnss_init();
bool     gnss_selfTest();
GnssData gnss_read();
unsigned long gps_charsProcessed();
