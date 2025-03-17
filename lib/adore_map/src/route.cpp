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
#include "adore_map/route.hpp"

namespace adore
{
namespace map
{

void
Route::add_route_section( Border& lane_to_add, const std::optional<MapPoint>& start_point, const std::optional<MapPoint>& end_point,
                          bool reverse = false )

{
  RouteSection next;
  if( reverse )
  {
    next.end_s   = lane_to_add.points.front().s;
    next.start_s = lane_to_add.points.back().s;
  }
  else
  {
    next.start_s = lane_to_add.points.front().s;
    next.end_s   = lane_to_add.points.back().s;
  }

  sections[lane_to_add.points[0].parent_id] = next;
}

double
Route::get_remaining_route_length() const
{
  double length = 0.0;
  for( const auto& [lane_id, section] : sections )
  {
    length += std::fabs( section.end_s - section.start_s );
  }
  return length;
}

} // namespace map
} // namespace adore
