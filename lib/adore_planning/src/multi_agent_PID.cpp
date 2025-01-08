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
#include "dynamics/vehicle_state.hpp"
#include "dynamics/integration.hpp"

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
    for (int i = 0; i < number_of_integration_steps; i++)
    {
        for (auto& pair : traffic_participant_set)
        {
            dynamics::VehicleStateDynamic next_state;
            dynamics::VehicleStateDynamic current_state;
            dynamics::VehicleCommand vehicle_command;
            if (pair.second.trajectory.has_value())
            {
                current_state = pair.second.trajectory.value().states.back();
            } else
            {
                current_state = pair.second.state;
            }
            
            double min_distance = 5.0;
            double current_trajectory_s = pair.second.route.value().get_s_at_state(current_state, min_distance);
            double target_distance = current_trajectory_s + 0.5 + 0.1 * current_state.vx;
            double goal_point_distance = 1e6;
            if (pair.second.goal_point.has_value())
            {
                goal_point_distance = pair.second.route.value().get_s_at_state(pair.second.goal_point.value(), min_distance) - current_trajectory_s + 1e-6;
            }

            double distance_from_closest_object = compute_distance_from_nearest_obstacle(traffic_participant_set, pair.first) - current_trajectory_s + 1e-6;
            
            math::Pose2d center_lane_target_pose = pair.second.route.value().get_pose_at_distance_along_route(target_distance);

            double error_direction = compute_error_direction(current_state, center_lane_target_pose);
            double error_lateral_distance = compute_error_lateral_distance(current_state, center_lane_target_pose);
            double error_speed = compute_error_speed(current_state.vx);
            double error_yaw = compute_error_yaw(current_state.yaw_angle, center_lane_target_pose.yaw);
            
            vehicle_command.acceleration = k_speed * error_speed - k_goal_point * 40.0 / goal_point_distance - k_repulsive_force * 10.0 / distance_from_closest_object;
            vehicle_command.steering_angle = k_direction * error_direction + k_yaw * error_yaw + k_distance * error_lateral_distance;

            if (vehicle_command.acceleration > limits.max_acceleration)
                vehicle_command.acceleration = limits.max_acceleration;
            if (vehicle_command.acceleration < limits.min_acceleration)
                vehicle_command.acceleration = limits.min_acceleration;
            if (vehicle_command.steering_angle > limits.max_steering_angle)
                vehicle_command.steering_angle = limits.max_steering_angle;
            if (vehicle_command.steering_angle < - limits.max_steering_angle)
                vehicle_command.steering_angle = - limits.max_steering_angle;
            
            next_state = dynamics::euler_step(current_state, vehicle_command, dt, wheelbase);

            pair.second.trajectory.value().states.push_back(next_state);
        }
    }
}

double 
MultiAgentPID::compute_error_direction( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d target_pose )
{
    double err_dir = 0.0;
    double psi = current_state.yaw_angle;
    double Dx = (target_pose.x - current_state.x);
    double Dy = (target_pose.y - current_state.y);
    double psi_objective = atan2(Dy, Dx);
    err_dir = psi_objective - psi;
    return err_dir;
}

double 
MultiAgentPID::compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d target_pose )
{
    double error_distance = 0.0;
    double Dx = (target_pose.x - current_state.x);
    double Dy = (target_pose.y - current_state.y);
    error_distance = Dx * cos(current_state.yaw_angle) + Dy * sin(current_state.yaw_angle);
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
    error_yaw = yaw_objective - current_yaw;
    return error_yaw;
}

double 
MultiAgentPID::compute_distance_from_nearest_obstacle( dynamics::TrafficParticipantSet& traffic_participant_set, int id_vehicle )
{
    double distance_from_closest_obstacle = 1e6;
    double distance = 1e6;
    double min_distance = 5.0;
    dynamics::TrafficParticipant reference_vehicle = traffic_participant_set.at(id_vehicle);
    for (auto& pair : traffic_participant_set)
    {
        if (pair.first == id_vehicle) {
            continue;  
        }
        dynamics::VehicleStateDynamic object_position;
        if (pair.second.trajectory.has_value())
        {
            object_position = pair.second.trajectory.value().states.back();
        } else
        {
            object_position = pair.second.state;
        }
        
        distance = pair.second.route.value().get_s_at_state(object_position, min_distance);
        if (distance_from_closest_obstacle > distance)
        {
            distance_from_closest_obstacle = distance;
        }
    }
    return distance_from_closest_obstacle;
}

} // namespace planner
} // namespace adore
