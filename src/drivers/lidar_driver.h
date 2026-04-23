#pragma once
#include "config.h"
#if HAS_LIDAR

#include "sensor_types.h"

bool      lidar_init();
bool      lidar_selfTest();
LidarData lidar_read();

#endif // HAS_LIDAR
