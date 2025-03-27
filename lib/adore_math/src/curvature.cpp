/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#include "adore_math/curvature.hpp"

namespace adore
{
namespace math
{

double
compute_curvature( double x1, double y1, double x2, double y2, double x3, double y3 )
{
  // Compute the curvature using the formula for three points
  double dx1 = x2 - x1;
  double dy1 = y2 - y1;
  double dx2 = x3 - x2;
  double dy2 = y3 - y2;

  double cross = dx1 * dy2 - dy1 * dx2;

  double ds1 = std::sqrt( dx1 * dx1 + dy1 * dy1 );
  double ds2 = std::sqrt( dx2 * dx2 + dy2 * dy2 );

  if( ds1 * ds2 == 0.0 )
    return 0.0;

  double sin_theta = cross / ( ds1 * ds2 );
  double curvature = sin_theta / ( ( ds1 + ds2 ) / 2.0 );

  return curvature;
}
} // namespace math
} // namespace adore
