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
    participant.trajectory = dynamics::Trajectory(); // to do only reset controllable participant trajectories
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

      double distance_from_closest_obstacle = std::numeric_limits<double>::max();

      if( !participant.route.has_value() || participant.route->center_lane.size() == 0 )
      {
        vehicle_command.acceleration   = 0.0;
        vehicle_command.steering_angle = 0.0;
        next_state                     = dynamics::euler_step( current_state, vehicle_command, dt, wheelbase );
        participant.trajectory->states.push_back( next_state );
        continue;
      }

      double current_trajectory_s = participant.route.value().get_s_at_state( current_state );
      double target_distance      = current_trajectory_s + 0.5 + 0.1 * current_state.vx;
      double goal_point_distance  = participant.route.value().center_lane.back().s - current_trajectory_s;

      auto object_dist_and_speed = compute_distance_from_nearest_obstacle( traffic_participant_set, id );

      math::Pose2d center_lane_target_pose = participant.route->get_pose_at_distance_along_route( target_distance );
      double       error_lateral_distance  = compute_error_lateral_distance( current_state, center_lane_target_pose );
      double       error_yaw               = compute_error_yaw( current_state.yaw_angle, center_lane_target_pose.yaw );


      vehicle_command.acceleration = -k_speed
                                   * ( current_state.vx
                                       - compute_idm_velocity( object_dist_and_speed.first, goal_point_distance,
                                                               object_dist_and_speed.second, current_state ) );

      vehicle_command.steering_angle = k_yaw * error_yaw + k_distance * error_lateral_distance;

      vehicle_command.clamp_within_limits( limits );

      next_state = dynamics::euler_step( current_state, vehicle_command, dt, wheelbase );

      if( next_state.vx < 0.0 )
      {
        next_state.vx = 0.0;
      }
      next_state.ax             = vehicle_command.acceleration;
      next_state.steering_angle = vehicle_command.steering_angle;

      participant.trajectory->states.push_back( next_state );
    }
  }
}

double
MultiAgentPID::compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose )
{
  double sin_yaw_ref = std::sin( target_pose.yaw );
  double cos_yaw_ref = std::cos( target_pose.yaw );

  double sin_yaw_current = std::sin( current_state.yaw_angle );
  double cos_yaw_current = std::cos( current_state.yaw_angle );

  double delta_x = current_state.x - target_pose.x; // Distance in the x direction (longitudinal)
  double delta_y = current_state.y - target_pose.y; // Distance in the y direction (lateral)

  double error_y = cos_yaw_ref * delta_y - sin_yaw_ref * delta_x; // Lateral error

  return -error_y;
}

double
MultiAgentPID::compute_idm_velocity( double dist_to_nearest_object, double distance_to_goal, double nearest_object_speed,
                                     const dynamics::VehicleStateDynamic& current_state )
{
  double min_distance_to_vehicle_ahead = 8.0;
  double desired_time_headway          = 3.0;

  double distance_for_idm = std::min( dist_to_nearest_object, distance_to_goal );

  if( distance_to_goal < dist_to_nearest_object )
  {
    min_distance_to_vehicle_ahead = 0;
  }

  double s_star = min_distance_to_vehicle_ahead + current_state.vx * desired_time_headway
                + current_state.vx * ( current_state.vx - nearest_object_speed )
                    / ( 2 * sqrt( desired_acceleration * desired_deceleration ) );

  double idm_velocity = current_state.vx
                      + desired_acceleration
                          * ( 1
                              - ( current_state.vx / max_speed ) * ( current_state.vx / max_speed ) * ( current_state.vx / max_speed )
                                  * ( current_state.vx / max_speed )
                              - ( s_star / distance_for_idm ) * ( s_star / distance_for_idm ) );

  return idm_velocity;
}

double
MultiAgentPID::compute_error_yaw( const double current_yaw, const double yaw_objective )
{
  double error_yaw = 0.0;
  error_yaw        = yaw_objective - current_yaw;
  error_yaw        = math::normalize_angle( error_yaw );
  return error_yaw;
}

std::pair<double, double>
MultiAgentPID::compute_distance_from_nearest_obstacle( dynamics::TrafficParticipantSet& traffic_participant_set, int id_vehicle )
{
  double           max_distance           = std::numeric_limits<double>::max();
  constexpr double lane_center_offset_max = 3.5;

  double closest_obstacle_distance = max_distance;
  double nearest_obstacle_speed    = 0.0;

  auto& reference_participant = traffic_participant_set[id_vehicle];

  for( auto& [id, other_participant] : traffic_participant_set )
  {
    if( id == id_vehicle )
    {
      continue;
    }

    // Get the state of the other participant
    const auto&                   trajectory   = other_participant.trajectory;
    dynamics::VehicleStateDynamic object_state = ( trajectory && !trajectory->states.empty() ) ? trajectory->states.back()
                                                                                               : other_participant.state;

    // Skip if the reference participant has no route
    if( !reference_participant.route.has_value() )
    {
      continue;
    }

    auto&  route            = reference_participant.route.value();
    double distance         = route.get_s_at_state( object_state );
    auto   pose_at_distance = route.get_pose_at_distance_along_route( distance );

    // Adjust distance if trajectory is available
    if( reference_participant.trajectory && !reference_participant.trajectory->states.empty() )
    {
      distance -= route.get_s_at_state( reference_participant.trajectory->states.back() );
    }

    // Skip if the offset from the lane center exceeds the maximum allowed
    double offset = math::distance_2d( object_state, pose_at_distance );
    if( offset > lane_center_offset_max || distance < 1.0 )
    {
      continue;
    }

    // Update the nearest obstacle if this one is closer
    if( distance < closest_obstacle_distance )
    {
      closest_obstacle_distance = distance;
      nearest_obstacle_speed    = object_state.vx;
    }
  }

  return { closest_obstacle_distance, nearest_obstacle_speed };
}

} // namespace planner
} // namespace adore
