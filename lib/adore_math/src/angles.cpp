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
normalize_angle( double angle )
{
  // Normalize heading error to the range [-pi, pi]
  while( angle > M_PI )
    angle -= 2.0 * M_PI;
  while( angle < -M_PI )
    angle += 2.0 * M_PI;
  return angle;
}
} // namespace math
} // namespace adore
