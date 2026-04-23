#pragma once
#include "config.h"
#if HAS_BARO

#include "sensor_types.h"

bool     baro_init();
bool     baro_selfTest();
void     baro_setSeaLevel(float hPa);
bool     baro_isCalibrated();       // true once an absolute MSL reference is set
float    baro_currentPressure();    // raw pressure for calibration
BaroData baro_read();

#endif // HAS_BARO
