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

            double target_distance = 0.5 + 0.1 * current_state.vx;
            
            math::Pose2d center_lane_target_pose = pair.second.route.value().get_pose_at_distance_along_route(target_distance);

            double error_direction = compute_error_direction(current_state, center_lane_target_pose);
            double error_lateral_distance = compute_error_lateral_distance(current_state, center_lane_target_pose);
            double error_speed = compute_error_speed(current_state.vx);
            double error_yaw = compute_error_yaw(current_state.yaw_angle, center_lane_target_pose.yaw);
            
            vehicle_command.acceleration = k_speed * error_speed;
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
MultiAgentPID::compute_error_direction(const dynamics::VehicleStateDynamic& current_state, const math::Pose2d target_pose)
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
MultiAgentPID::compute_error_lateral_distance(const dynamics::VehicleStateDynamic& current_state, const math::Pose2d target_pose)
{
    double error_distance = 0.0;
    double Dx = (target_pose.x - current_state.x);
    double Dy = (target_pose.y - current_state.y);
    error_distance = Dx * cos(current_state.yaw_angle) + Dy * sin(current_state.yaw_angle);
    return error_distance;
}

double
MultiAgentPID::compute_error_speed(const double speed)
{
    double error_speed = max_speed - speed;
    return error_speed;
}

double
MultiAgentPID::compute_error_yaw(const double current_yaw, const double yaw_objective)
{
    double error_yaw = 0.0;
    error_yaw = yaw_objective - current_yaw;
    return error_yaw;
}

} // namespace planner
} // namespace adore
