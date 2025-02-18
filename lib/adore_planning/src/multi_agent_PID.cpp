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
 *    Marko Mizdrak
 ********************************************************************************/

#include "planning/multi_agent_PID.hpp"

#include "adore_math/curvature.hpp"
#include "adore_math/point.h"
#include "adore_math/spline.h"

#include "dynamics/integration.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace planner
{

MultiAgentPID::MultiAgentPID() = default;

void
MultiAgentPID::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "wheelbase" )
      wheelbase = value;
    else if( name == "max_speed" )
      max_speed = value;
    else if( name == "desired_acceleration" )
      desired_acceleration = value;
    else if( name == "desired_deceleration" )
      desired_deceleration = value;
    else if( name == "max_lateral_acceleration" )
      max_lateral_acceleration = value;
    else if( name == "number_of_integration_steps" )
      number_of_integration_steps = value;
    else if( name == "k_speed" )
      k_speed = value;
    else if( name == "k_yaw" )
      k_yaw = value;
    else if( name == "k_distance" )
      k_distance = value;
    else if( name == "k_goal_point" )
      k_goal_point = value;
    else if( name == "k_repulsive_force" )
      k_repulsive_force = value;
    else if( name == "dt" )
      dt = value;
  }
}

void
MultiAgentPID::plan_trajectories( dynamics::TrafficParticipantSet& traffic_participant_set, const dynamics::VehicleCommandLimits& limits )
{
  for( auto& [id, participant] : traffic_participant_set.participants )
  {
    participant.trajectory = dynamics::Trajectory();
  }

  for( int i = 0; i < number_of_integration_steps; ++i )
  {
    for( auto& [id, participant] : traffic_participant_set.participants )
    {
      dynamics::VehicleStateDynamic next_state;
      dynamics::VehicleStateDynamic current_state = participant.trajectory && !participant.trajectory->states.empty()
                                                    ? participant.trajectory->states.back()
                                                    : participant.state;

      if( !participant.route || participant.route->center_lane.empty() )
      {
        next_state = dynamics::euler_step( current_state, {}, dt, wheelbase );
        participant.trajectory->states.push_back( next_state );
        continue;
      }

      double current_trajectory_s = participant.route->get_s_at_state( current_state );
      double target_distance      = current_trajectory_s + 0.5 + 0.1 * current_state.vx;
      double goal_point_distance  = participant.route->center_lane.back().s - current_trajectory_s;

      auto [closest_obstacle_distance, obstacle_speed] = compute_distance_from_nearest_obstacle( traffic_participant_set, id );

      math::Pose2d target_pose   = participant.route->get_pose_at_distance_along_route( target_distance );
      double       error_lateral = compute_error_lateral_distance( current_state, target_pose );
      double       error_yaw     = compute_error_yaw( current_state.yaw_angle, target_pose.yaw );

      dynamics::VehicleCommand vehicle_command;
      vehicle_command.acceleration
        = -k_speed
        * ( current_state.vx - compute_idm_velocity( closest_obstacle_distance, goal_point_distance, obstacle_speed, current_state ) );
      vehicle_command.steering_angle = k_yaw * error_yaw + k_distance * error_lateral;

      vehicle_command.clamp_within_limits( limits );

      next_state                = dynamics::euler_step( current_state, vehicle_command, dt, wheelbase );
      next_state.vx             = std::max( 0.0, next_state.vx );
      next_state.ax             = vehicle_command.acceleration;
      next_state.steering_angle = vehicle_command.steering_angle;

      participant.trajectory->states.push_back( next_state );
    }
  }
}

double
MultiAgentPID::compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose )
{
  double sin_yaw = std::sin( target_pose.yaw );
  double cos_yaw = std::cos( target_pose.yaw );

  double delta_x = current_state.x - target_pose.x;
  double delta_y = current_state.y - target_pose.y;

  return -( cos_yaw * delta_y - sin_yaw * delta_x );
}

double
MultiAgentPID::compute_idm_velocity( double obstacle_distance, double goal_distance, double obstacle_speed,
                                     const dynamics::VehicleStateDynamic& current_state )
{
  double min_distance = 8.0;
  double time_headway = 3.0;

  double effective_distance = std::min( obstacle_distance, goal_distance );
  if( goal_distance < obstacle_distance )
    min_distance = 0.0;

  double s_star = min_distance + current_state.vx * time_headway
                + current_state.vx * ( current_state.vx - obstacle_speed )
                    / ( 2 * std::sqrt( desired_acceleration * desired_deceleration ) );

  return current_state.vx
       + desired_acceleration * ( 1 - std::pow( current_state.vx / max_speed, 4 ) - std::pow( s_star / effective_distance, 2 ) );
}

double
MultiAgentPID::compute_error_yaw( double current_yaw, double target_yaw )
{
  return math::normalize_angle( target_yaw - current_yaw );
}

std::pair<double, double>
MultiAgentPID::compute_distance_from_nearest_obstacle( dynamics::TrafficParticipantSet& traffic_participant_set, int vehicle_id )
{

  constexpr double lane_offset_max  = 3.5;
  double           closest_distance = std::numeric_limits<double>::max();
  double           obstacle_speed   = 0.0;

  auto& ref_participant = traffic_participant_set.participants[vehicle_id];

  for( const auto& [id, other_participant] : traffic_participant_set.participants )
  {
    if( id == vehicle_id )
      continue;

    const auto&                   trajectory   = other_participant.trajectory;
    dynamics::VehicleStateDynamic object_state = ( trajectory && !trajectory->states.empty() ) ? trajectory->states.back()
                                                                                               : other_participant.state;

    if( !ref_participant.route )
      continue;

    auto&  route            = ref_participant.route.value();
    double distance         = route.get_s_at_state( object_state );
    auto   pose_at_distance = route.get_pose_at_distance_along_route( distance );

    if( ref_participant.trajectory && !ref_participant.trajectory->states.empty() )
    {
      distance -= route.get_s_at_state( ref_participant.trajectory->states.back() );
    }

    double offset = math::distance_2d( object_state, pose_at_distance );
    if( offset > lane_offset_max || distance < 1.0 )
      continue;

    if( distance < closest_distance )
    {
      closest_distance = distance;
      obstacle_speed   = object_state.vx;
    }
  }

  return { closest_distance, obstacle_speed };
}

} // namespace planner
} // namespace adore
