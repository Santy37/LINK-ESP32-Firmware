/* Waypoint Computation — destination point from observer + bearing + range
 *
 * This is the C equivalent of hud-app/src/lib/geo.ts → destinationPoint()
 */

#pragma once

#include <cmath>
#include "config.h"

struct LatLon {
  double lat;
  double lon;
};

/**
 * Compute the target lat/lon given an observer position, bearing, and range.
 * Same formula used on the phone side — results must agree.
 */
inline LatLon destinationPoint(double latDeg, double lonDeg,
                                float bearingDeg, float distanceM) {
  const double R    = cfg::EARTH_RADIUS_M;
  const double lat1 = latDeg  * M_PI / 180.0;
  const double lon1 = lonDeg  * M_PI / 180.0;
  const double brng = bearingDeg * M_PI / 180.0;
  const double dr   = distanceM / R;

  double lat2 = asin(sin(lat1) * cos(dr) + cos(lat1) * sin(dr) * cos(brng));
  double lon2 = lon1 + atan2(sin(brng) * sin(dr) * cos(lat1),
                              cos(dr) - sin(lat1) * sin(lat2));

  LatLon result;
  result.lat = lat2 * 180.0 / M_PI;
  result.lon = lon2 * 180.0 / M_PI;
  return result;
}
