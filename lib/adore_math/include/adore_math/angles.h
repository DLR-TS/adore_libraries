/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once
#include <math.h>

namespace adore
{
namespace math
{

double to_degrees( double radians );
double to_radians( double degrees );
// Normalize heading error to the range [-pi, pi]
double normalize_angle( double angle );

double latlon_deg_to_utm_rad( double latlon_degrees );
double utm_rad_to_latlon_deg( double utm_radians );

template<typename PointType>
double
compute_yaw( const PointType& from, const PointType& to )
{
  double delta_x = to.x - from.x;
  double delta_y = to.y - from.y;
  return std::atan2( delta_y, delta_x ); // Returns angle in radians
}

// get yaw from quaternion (since tf2 is annoying sometimes)
template<typename Quaternion>
static inline double
get_yaw( const Quaternion& q )
{

  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;

  double siny_cosp = 2.0 * ( w * z + x * y );
  double cosy_cosp = 1.0 - 2.0 * ( y * y + z * z );
  double yaw       = std::atan2( siny_cosp, cosy_cosp );

  return yaw;
}

template<typename Quaternion>
static inline double
get_roll( const Quaternion& q )
{
  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;

  double sinr_cosp = 2.0 * ( w * x + y * z );
  double cosr_cosp = 1.0 - 2.0 * ( x * x + y * y );
  double roll      = std::atan2( sinr_cosp, cosr_cosp );

  return roll;
}

template<typename Quaternion>
static inline double
get_pitch( const Quaternion& q )
{
  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;

  double sinp = 2.0 * ( w * y - z * x );
  double pitch;

  if( std::abs( sinp ) >= 1 )
    pitch = std::copysign( M_PI / 2, sinp ); // Use 90 degrees if out of range
  else
    pitch = std::asin( sinp );

  return pitch;
}
} // namespace math
} // namespace adore
