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
MultiAgentPID::MultiAgentPID() {}

void
MultiAgentPID::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "wheelbase" )
      wheelbase = value;
    if( name == "max_speed" )
      max_speed = value;
    if( name == "desired_acceleration" )
      desired_acceleration = value;
    if( name == "desired_deceleration" )
      desired_deceleration = value;
    if( name == "max_lateral_acceleration" )
      max_lateral_acceleration = value;
    if( name == "number_of_integration_steps" )
      number_of_integration_steps = value;
    if( name == "k_direction" )
      k_direction = value;
    if( name == "k_speed" )
      k_speed = value;
    if( name == "k_yaw" )
      k_yaw = value;
    if( name == "k_distance" )
      k_distance = value;
    if( name == "k_goal_point" )
      k_goal_point = value;
    if( name == "k_repulsive_force" )
      k_repulsive_force = value;
  }
}

void
MultiAgentPID::plan_trajectories( dynamics::TrafficParticipantSet& traffic_participant_set, const dynamics::VehicleCommandLimits& limits )
{
  // reset trajectories
  for( auto& [id, participant] : traffic_participant_set )
  {
    participant.trajectory = dynamics::Trajectory();
  }
  for( int i = 0; i < number_of_integration_steps; i++ )
  {
    for( auto& [id, participant] : traffic_participant_set )
    {
      dynamics::VehicleStateDynamic next_state;
      dynamics::VehicleStateDynamic current_state;
      dynamics::VehicleCommand      vehicle_command;

      if( !participant.trajectory.has_value() )
      {
        dynamics::Trajectory trajectory_default;
        participant.trajectory = trajectory_default;
      }

      if( participant.trajectory->states.size() > 0 )
      {
        current_state = participant.trajectory.value().states.back();
      }
      else
      {
        current_state = participant.state;
      }

      double eps                                            = 1e-6;
      double distance_to_goal_where_deceleration_starts     = 40.0;
      double distance_to_obstacle_where_deceleration_starts = 10.0;
      double min_distance                                   = 5.0;
      double current_trajectory_s                           = 0.0;

      if( participant.route.has_value() )
      {
        current_trajectory_s = participant.route.value().get_s_at_state( current_state, min_distance );
      }
      else
      {
        vehicle_command.acceleration   = 0.0;
        vehicle_command.steering_angle = 0.0;
        next_state                     = dynamics::euler_step( current_state, vehicle_command, dt, wheelbase );
        participant.trajectory->states.push_back( next_state );
        continue;
      }

      double target_distance     = current_trajectory_s + 0.5 + 0.1 * current_state.vx;
      double goal_point_distance = 1e6;

      goal_point_distance = participant.route.value().center_lane.back().s - current_trajectory_s + eps;

      double distance_from_closest_obstacle = compute_distance_from_nearest_obstacle( traffic_participant_set, id ) - current_trajectory_s
                                            + eps;
      math::Pose2d center_lane_target_pose = participant.route->get_pose_at_distance_along_route( target_distance );
      double       error_direction         = compute_error_direction( current_state, center_lane_target_pose );
      double       error_lateral_distance  = compute_error_lateral_distance( current_state, center_lane_target_pose );
      double       error_speed             = compute_error_speed( current_state.vx );
      double       error_yaw               = compute_error_yaw( current_state.yaw_angle, center_lane_target_pose.yaw );

      vehicle_command.acceleration = k_speed * error_speed - k_goal_point * distance_to_goal_where_deceleration_starts / goal_point_distance
                                   - k_repulsive_force * distance_to_obstacle_where_deceleration_starts / distance_from_closest_obstacle;
      vehicle_command.steering_angle = k_direction * error_direction + k_yaw * error_yaw + k_distance * error_lateral_distance;

      vehicle_command.clamp_within_limits( limits );

      next_state = dynamics::euler_step( current_state, vehicle_command, dt, wheelbase );

      if( next_state.vx < 0.0 )
      {
        next_state.vx = 0.0;
      }

      participant.trajectory->states.push_back( next_state );
    }
  }
}

double
MultiAgentPID::compute_error_direction( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose )
{
  double err_dir       = 0.0;
  double psi           = current_state.yaw_angle;
  double dx            = ( target_pose.x - current_state.x );
  double dy            = ( target_pose.y - current_state.y );
  double psi_objective = atan2( dy, dx );
  err_dir              = psi_objective - psi;
  err_dir              = math::normalize_angle( err_dir );
  return err_dir;
}

double
MultiAgentPID::compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose )
{
  double error_distance = 0.0;
  double dx             = ( target_pose.x - current_state.x );
  double dy             = ( target_pose.y - current_state.y );
  error_distance        = dx * cos( current_state.yaw_angle ) + dy * sin( current_state.yaw_angle );
  return error_distance;
}

double
MultiAgentPID::compute_error_speed( const double speed )
{
  double error_speed = max_speed - speed;
  return error_speed;
}

double
MultiAgentPID::compute_error_yaw( const double current_yaw, const double yaw_objective )
{
  double error_yaw = 0.0;
  error_yaw        = yaw_objective - current_yaw;
  error_yaw        = math::normalize_angle( error_yaw );
  return error_yaw;
}

double
MultiAgentPID::compute_distance_from_nearest_obstacle( dynamics::TrafficParticipantSet& traffic_participant_set, int id_vehicle )
{
  double distance_from_closest_obstacle = std::numeric_limits<double>::max();
  double distance                       = std::numeric_limits<double>::max();
  bool   within_the_lane;
  double min_distance = 1.5;

  for( auto& [id, participant] : traffic_participant_set )
  {
    if( id == id_vehicle )
    {
      continue;
    }
    dynamics::VehicleStateDynamic object_position;
    if( participant.trajectory.has_value() && participant.trajectory->states.size() > 0 )
    {
      object_position = participant.trajectory.value().states.back();
    }
    else
    {
      object_position = participant.state;
    }
    if( participant.route )
      distance = participant.route.value().get_s_at_state( object_position, min_distance );

    if( distance < 1e-6 )
    {
      continue;
    }

    if( distance_from_closest_obstacle > distance )
    {
      distance_from_closest_obstacle = distance;
    }
  }
  return distance_from_closest_obstacle;
}

} // namespace planner
} // namespace adore
