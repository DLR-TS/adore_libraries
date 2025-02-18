/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#include "adore_math/angles.h"

namespace adore
{
namespace math
{

double
latlon_deg_to_utm_rad( double latlon_degrees )
{
  return to_radians( 90 - latlon_degrees );
}

double
utm_rad_to_latlon_deg( double utm_radians )
{
  return to_degrees( M_PI / 2 - utm_radians );
}

double
to_degrees( double radians )
{
  double degrees = radians * ( 180.0 / M_PI );
  degrees        = fmod( degrees, 360.0 );
  if( degrees < 0 )
    degrees += 360.0; // Ensure it's within [0, 360)
  return degrees;
}

double
to_radians( double degrees )
{
  return normalize_angle( degrees * ( M_PI / 180.0 ) );
}

double
normalize_angle( double angle )
{
  angle = fmod( angle + M_PI, 2.0 * M_PI );
  if( angle < 0 )
    angle += 2.0 * M_PI;
  return angle - M_PI;
}
} // namespace math
} // namespace adore
