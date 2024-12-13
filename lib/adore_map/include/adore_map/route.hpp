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
#include <stdlib.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "adore_map/lane.hpp"
#include "adore_map/quadtree.hpp"
#include "adore_map/r2s_parser.h"
#include "adore_map/road_graph.hpp"
#include "adore_math/distance.h"
#include "adore_math/point.h"

namespace adore
{
namespace map
{

struct Route
{
  std::deque<MapPoint> center_lane;
  std::deque<size_t>   lane_id_route;
  adore::math::Point2d start;
  adore::math::Point2d destination;

  // Add points to the route
  void add_lane_center( Border& points, const std::optional<MapPoint>& start_point, const std::optional<MapPoint>& end_point,
                        bool reverse );

  // get length of route
  double get_remaining_route_length();

  // Helper functions
  template<typename State>
  double
  get_s_at_state( const State& state, double& min_distance )
  {
    {
      if( center_lane.empty() )
      {
        // Route is empty
        return 0.0;
      }

      // Initialize minimum distance and corresponding s value
      double s_at_min_distance = 0.0;

      // Iterate over the route points to find the nearest point
      for( const auto& point : center_lane )
      {
        double distance = adore::math::distance_2d( state, point );
        if( distance < min_distance )
        {
          min_distance      = distance;
          s_at_min_distance = point.s;
        }
      }

      return s_at_min_distance;
    }
  }

  template<typename State>
  void
  trim_route_up_to_state( const State& state )
  {
    double min_dist = std::numeric_limits<double>::max();
    double s        = get_s_at_state( state, min_dist );

    while( !center_lane.empty() && center_lane.front().s < s )
    {
      if( !lane_id_route.empty() && lane_id_route.front() == center_lane.front().parent_id )
        lane_id_route.pop_front();
      center_lane.pop_front();
    }

    for( auto& point : center_lane )
    {
      point.s -= s;
    }
  }

  Route
  get_shortened_route( const double desired_length )
  {
    Route cut_route;
    cut_route.start       = start;
    cut_route.destination = destination;

    if( center_lane.empty() )
    {
      return cut_route;
    }

    double s       = 0;
    double start_s = center_lane.front().s;
    for( const auto& p : center_lane )
    {
      if( p.s - start_s >= desired_length )
        break;
      cut_route.center_lane.push_back( p );
      if( cut_route.lane_id_route.empty() || cut_route.lane_id_route.back() != p.parent_id )
        cut_route.lane_id_route.push_back( p.parent_id );
    }
    return cut_route;
  }

  void
  interpolate_center_lane( double desired_spacing )
  {
    if( center_lane.empty() || center_lane.size() == 1 )
    {
      // No interpolation needed
      return;
    }

    std::deque<MapPoint> interpolated_lane;

    // Initialize with the first point
    MapPoint previous_point = center_lane.front();
    interpolated_lane.push_back( previous_point );

    double accumulated_distance = 0.0;

    for( size_t i = 1; i < center_lane.size(); ++i )
    {
      const MapPoint& current_point = center_lane[i];

      // Calculate distance between previous_point and current_point
      double dx               = current_point.x - previous_point.x;
      double dy               = current_point.y - previous_point.y;
      double segment_distance = std::sqrt( dx * dx + dy * dy );

      if( segment_distance <= 0.0 )
      {
        // Identical points, skip to avoid division by zero
        continue;
      }

      double direction_x = dx / segment_distance;
      double direction_y = dy / segment_distance;

      double remaining_distance = desired_spacing - accumulated_distance;

      while( segment_distance >= remaining_distance )
      {
        // Calculate the new point's coordinates
        double new_x = previous_point.x + direction_x * remaining_distance;
        double new_y = previous_point.y + direction_y * remaining_distance;
        double new_s = previous_point.s + remaining_distance;

        // Create and add the new interpolated MapPoint
        MapPoint new_point( new_x, new_y, previous_point.parent_id );
        new_point.s = new_s;
        interpolated_lane.push_back( new_point );

        // Update for the next iteration
        previous_point        = new_point;
        accumulated_distance  = 0.0;
        segment_distance     -= remaining_distance;

        // Update direction in case the direction changes in the new segment
        direction_x = ( current_point.x - previous_point.x ) / segment_distance;
        direction_y = ( current_point.y - previous_point.y ) / segment_distance;
      }

      // Accumulate the remaining distance
      accumulated_distance += segment_distance;

      // Set previous_point to current_point for the next segment
      previous_point = current_point;
      interpolated_lane.push_back( previous_point );
    }

    // Replace the original center_lane with the interpolated_lane
    center_lane = std::move( interpolated_lane );
  }
};


} // namespace map
} // namespace adore
