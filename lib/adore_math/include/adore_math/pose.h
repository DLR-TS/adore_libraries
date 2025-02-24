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
 *    Giovanni Lucente
 ********************************************************************************/
#pragma once

namespace adore

{
namespace math
{


struct Pose2d
{
  double x, y, yaw;
  Pose2d() {};
  Pose2d( double x_, double y_, double yaw_ ) :
    x( x_ ),
    y( y_ ),
    yaw( yaw_ ) {};
};
} // namespace math
} // namespace adore