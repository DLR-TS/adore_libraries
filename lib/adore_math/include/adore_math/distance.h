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

template<typename PointType, typename OtherPointType>
double
squared_distance_2d( const PointType& a, const OtherPointType& b )
{
  return ( a.x - b.x ) * ( a.x - b.x ) + ( a.y - b.y ) * ( a.y - b.y );
}

template<typename PointType, typename OtherPointType>
double
distance_2d( const PointType& a, const OtherPointType& b )
{
  return std::sqrt( squared_distance_2d( a, b ) );
}
} // namespace math
} // namespace adore
