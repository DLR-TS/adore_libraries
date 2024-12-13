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
#include <cmath>

namespace adore
{
namespace math
{


double compute_curvature( double x1, double y1, double x2, double y2, double x3, double y3 );

template<typename PointType>
double
compute_curvate( const PointType& p1, const PointType& p2, const PointType& p3 )
{
  return compute_curvature( p1.x, p1.y, p2.x, p2.y, p3.x, p3.y );
}
} // namespace math
} // namespace adore
