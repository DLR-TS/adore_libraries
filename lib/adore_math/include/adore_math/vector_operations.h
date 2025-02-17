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

#include "adore_math/point.h"
#include "adore_math/vector.h"

namespace adore
{
namespace math
{

template<typename Vector2dType, typename OtherVector2dType>
double
scalar_product( const Vector2dType& a, const OtherVector2dType& b )
{
  return ( a.x * b.x ) + ( a.y * b.y );
}

template<typename Vector2dType, typename OtherVector2dType>
double
cross_product( const Vector2dType& a, const OtherVector2dType& b )
{
  return a.x * b.y - a.y * b.x;
}

template<typename Vector2dType>
double
get_l2_norm( const Vector2dType& a )
{
  return std::sqrt( a.x * a.x + a.y * a.y );
}

template<typename Vector2dType>
Vector2d
get_versor( const Vector2dType& a )
{
  Vector2d versor;
  double   l2_norm_a = get_l2_norm( a );
  versor.x           = a.x / l2_norm_a;
  versor.y           = a.y / l2_norm_a;
  return versor;
}

template<typename PointType, typename OtherPointType>
Vector2d
get_vector_from_a_to_b( const PointType a, const OtherPointType b )
{
  Vector2d vector;
  vector.x = a.x - b.x;
  vector.y = a.y - b.y;
  return vector;
}

Vector2d
get_versor_from_angle( const double angle )
{
  Vector2d vector;
  vector.x = std::cos( angle );
  vector.y = std::sin( angle );
  return vector;
}

template<typename Vector2dType, typename OtherVector2dType>
double
get_angle_between_two_vectors( const Vector2dType& a, const OtherVector2dType& b )
{
  double ab     = scalar_product( a, b );
  double norm_a = get_l2_norm( a );
  double norm_b = get_l2_norm( b );

  double axb = cross_product( a, b );

  // Avoid division by zero
  if( norm_a == 0.0 || norm_b == 0.0 )
    return 0.0;

  // Compute the cosine of the angle
  double cos_theta = ab / ( norm_a * norm_b );

  // Clamp the value to avoid floating-point precision issues
  cos_theta = std::max( -1.0, std::min( 1.0, cos_theta ) );

  // Compute the unsigned angle in radians
  double angle = std::acos( cos_theta );

  // Use cross product to determine sign (-pi to +pi range)
  return ( axb < 0 ) ? -angle : angle;
}

template<typename Vector2dType>
Vector2d
get_perpendicular_versor( const Vector2dType& v, bool clockwise = false )
{
  Vector2d vector;
  if( clockwise ) // Clockwise (-90°)
  {
    vector.x = v.y;
    vector.y = -v.x;
  }
  else // Counterclockwise (+90°)
  {
    vector.x = -v.y;
    vector.y = v.x;
  }
  return vector;
}

} // namespace math
} // namespace adore
