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
#include "adore_map/map.hpp"
#include "adore_map/quadtree.hpp"
#include "adore_map/r2s_parser.h"
#include "adore_map/road_graph.hpp"
#include "adore_math/distance.h"
#include "adore_math/point.h"
#include "adore_math/pose.h"

namespace adore
{
namespace map
{

// new struct
struct RouteSection
{
  double start_s;
  double end_s;
};

struct Route
{
  // lane id to routesection
  std::unordered_map<size_t, RouteSection> sections;
  std::shared_ptr<Map>                     map;
  adore::math::Point2d                     start;
  adore::math::Point2d                     destination;
  std::map<double, MapPoint&>              center_lane;


  // Add points to the route
  void add_route_section( Border& points, const std::optional<MapPoint>& start_point, const std::optional<MapPoint>& end_point,
                          bool reverse );

  // get length of route
  double get_remaining_route_length() const;

  // get distance to object along route and if the object is within the lane
  template<typename State>
  std::pair<bool, double>
  get_distance_along_route( const State& object_position ) const
  {
    // TODO
  }

  template<typename State>
  void
  trim_route_up_to_state( const State& state )
  {
    // TODO
  }

  std::deque<MapPoint>
  get_shortened_route( double desired_length )
  {
    std::deque<MapPoint> result;
    for( auto& [s, point] : center_lane )
    {
      if( s <= desired_length )
      {
        result.push_back( point );
      }
      else
        break;
    }
    return result;
  }

  adore::math::Pose2d
  get_pose_at_distance_along_route( double distance ) const
  {
    adore::math::Pose2d pose; // defaults to (0,0,yaw=0)
    if( center_lane.empty() )
    {
      return pose;
    }
    if( center_lane.size() == 1 )
    {
      const auto& sp = center_lane.begin()->second;
      pose.x         = sp.x;
      pose.y         = sp.y;
      return pose;
    }

    // Find the first key >= distance
    auto upper_it = center_lane.lower_bound( distance );

    auto   lower_it = upper_it;
    double frac     = 0.0; // default

    if( upper_it == center_lane.end() )
    {
      upper_it--;
      lower_it = std::prev( upper_it );
      frac     = 1.0;
    }
    else if( upper_it == center_lane.begin() )
    {
      upper_it++;
      frac = 0.0;
    }
    else
    {
      // Normal case: we have an iterator in between begin() and end().
      lower_it--;
      double s1    = lower_it->first;
      double s2    = upper_it->first;
      double denom = ( s2 - s1 );
      frac         = ( std::fabs( denom ) < 1e-9 ) ? 0.0 : ( distance - s1 ) / denom;
    }

    // Now do the interpolation using lower_it and upper_it, plus frac
    double s1 = lower_it->first;
    double s2 = upper_it->first;

    double x1 = lower_it->second.x;
    double y1 = lower_it->second.y;
    double x2 = upper_it->second.x;
    double y2 = upper_it->second.y;

    // Interpolate x,y
    pose.x = x1 + frac * ( x2 - x1 );
    pose.y = y1 + frac * ( y2 - y1 );

    // Compute yaw from direction (x1->x2, y1->y2)
    double dx = x2 - x1;
    double dy = y2 - y1;
    if( !( std::fabs( dx ) < 1e-9 && std::fabs( dy ) < 1e-9 ) )
    {
      pose.yaw = std::atan2( dy, dx );
    }

    return pose;
  }

  template<typename StartPoint, typename EndPoint>
  Route( const StartPoint& start, const EndPoint& end, const Map& reference_map )
  {
    start.x       = start.x;
    start.y       = start.y;
    destination.x = end.x;
    destination.y = end.y;
    map           = reference_map;

    double route_cumulative_s = 0;

    // Find nearest start and end points using the quadtree
    double min_start_dist      = std::numeric_limits<double>::max();
    auto   nearest_start_point = map.quadtree.get_nearest_point( start, min_start_dist );
    double min_end_dist        = std::numeric_limits<double>::max();
    auto   nearest_end_point   = map.quadtree.get_nearest_point( end, min_end_dist );

    if( nearest_start_point && nearest_end_point )
    {
      size_t start_lane_id = nearest_start_point->parent_id;
      size_t end_lane_id   = nearest_end_point->parent_id;

      // Find the best path between the start and end lanes
      auto lane_id_route = map.lane_graph.get_best_path( start_lane_id, end_lane_id );

      // Iterate over the route and process each lane
      for( size_t i = 0; i < lane_id_route.size(); ++i )
      {
        add_route_section( lanmap.lanes.at( route.lane_id_route[i] )->borders.center, nearest_start_point, nearest_end_point,
                           lane->left_of_reference );
      }

      initialize_center_lane();
    }
    return route;
  }

  void
  initialize_center_lane()
  {
    center_lane.clear();
    if( !map )
    {
      return;
    }

    // Go through each RouteSection, gather center points from that lane in [start_s, end_s]
    for( const auto& [lane_id, section] : sections )
    {
      auto lane_it = map->lanes.find( lane_id );
      if( lane_it == map->lanes.end() )
        continue;

      std::shared_ptr<Lane> lane_ptr = lane_it->second;
      if( !lane_ptr )
        continue;

      const auto& cpoints = lane_ptr->borders.center.interpolated_points;
      bool        reverse = section.end_s < section.start_s;
      double      start_s = reverse ? section.end_s : section.start_s;
      double      end_s   = reverse ? section.start_s : section.end_s;

      for( size_t i = 0; i < cpoints.size(); ++i )
      {
        auto point = cpoints[reverse ? cpoints.size() - i - 1 : i];

        if( point.s >= start_s && point.s <= end_s )
        {
          center_lane[point.s] = point;
        }
      }
    }
  }
};

} // namespace map
} // namespace adore
