#pragma once
#include "config.h"
#if HAS_IMU

#include "sensor_types.h"

bool    imu_init();
bool    imu_selfTest();
ImuData imu_read();

#endif // HAS_IMU
