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
Route::add_lane_center( Border& lane_to_add, const std::optional<MapPoint>& start_point, const std::optional<MapPoint>& end_point,
                        bool reverse = false )

{
  const auto& points_to_add = lane_to_add.interpolated_points;

  bool adding_points = reverse ? points_to_add.back().parent_id != start_point->parent_id
                               : points_to_add.front().parent_id != start_point->parent_id;

  for( size_t i = 0; i < points_to_add.size(); ++i )
  {
    auto point = points_to_add[reverse ? points_to_add.size() - i - 1 : i];

    // Start adding points after the start point
    if( !adding_points && point == *start_point )
    {
      adding_points = true;
    }
    if( !adding_points )
      continue;

    // Update the point's 's' value based on the last point added to center_lane
    if( !center_lane.empty() )
    {
      point.s = center_lane.back().s + adore::math::distance_2d( point, center_lane.back() );
    }
    else
    {
      // Initialize 's' for the first point added
      point.s = 0.0;
    }

    center_lane.push_back( point );

    // Stop adding points once we get to the end
    if( end_point->parent_id == point.parent_id && point == *end_point )
      break;
  }
}

double
Route::get_remaining_route_length() const
{
  if( center_lane.size() < 2 )
    return 0;
  return center_lane.back().s - center_lane.front().s;
}

} // namespace map
} // namespace adore
