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
    if( name == "number_integration_nodes" )
      number_integration_nodes = value;
  }
}

dynamics::TrafficParticipantSet
MultiAgentPID::plan_trajectories( const dynamics::TrafficParticipantSet& traffic_participant_set,
                                    const dynamics::VehicleCommandLimits& limits )
{
    dynamics::TrafficParticipantSet PID_planned_traffic = traffic_participant_set;
    double k_dir = 0.6;
    double k_speed = 2.0;
    double k_heading = 0.1;

    for (int i = 0; i < number_integration_nodes; i++)
    {
        for (auto& pair : PID_planned_traffic)
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
            
            math::Point2d center_lane_reference_point = MultiAgentPID::compute_center_lane_reference_point(3.0, pair.second.route.value());
            double reference_heading = MultiAgentPID::compute_center_lane_reference_heading(3.0, pair.second.route.value());
            
            double error_direction = MultiAgentPID::compute_error_direction(current_state.x, current_state.y, current_state.yaw_angle,
                                                                            center_lane_reference_point.x, center_lane_reference_point.y);
            double error_distance = MultiAgentPID::compute_error_distance(current_state.x, current_state.y, current_state.yaw_angle,
                                                                            center_lane_reference_point.x, center_lane_reference_point.y);
            double error_speed = MultiAgentPID::compute_error_speed(current_state.vx);
            double error_heading = MultiAgentPID::compute_error_heading(current_state.yaw_angle, reference_heading);
            
            vehicle_command.acceleration = k_speed * error_speed;
            vehicle_command.steering_angle = k_dir * error_direction + k_heading * error_heading;

            next_state = dynamics::euler_step(current_state, vehicle_command, dt, wheelbase);

            pair.second.trajectory.value().states.push_back(next_state);
        }
    }
    
    return PID_planned_traffic;
}

double 
MultiAgentPID::compute_error_direction(const double x_position, const double y_position, const double heading,
                                     const double x_objective, const double y_objective)
{
    double err_dir = 0.0;
    double psi = heading;
    double Dx = (x_objective - x_position);
    double Dy = (y_objective - y_position);
    double psi_objective = atan2(Dy, Dx);
    err_dir = psi_objective - psi;
    return err_dir;
}

double 
MultiAgentPID::compute_error_distance(const double x_position, const double y_position, const double heading, 
                                const double x_objective, const double y_objective)
{
    double error_distance = 0.0;
    double Dx = (x_objective - x_position);
    double Dy = (y_objective - y_position);
    error_distance = Dx * cos(heading) + Dy * sin(heading);
    return error_distance;
}

double
MultiAgentPID::compute_error_speed(const double speed)
{
    double error_speed = max_speed - speed;
    return error_speed;
}

double
MultiAgentPID::compute_error_heading(const double heading, const double heading_objective)
{
    double error_heading = 0.0;
    error_heading = heading_objective - heading;
    return error_heading;
}

double
MultiAgentPID::compute_center_lane_reference_heading(const double distance, map::Route& route)
{
    double reference_heading = 0.0;
    math::Point2d reference_point;
    math::Point2d next_reference_point;
    for (int i = 0; i < route.center_lane.size() - 1; i++)
    {
        if (route.center_lane[i].s > distance)
        {
            reference_point.x = route.center_lane[i].x;
            reference_point.y = route.center_lane[i].y;
            next_reference_point.x = route.center_lane[i + 1].x;
            next_reference_point.y = route.center_lane[i + 1].y;
            reference_heading = std::atan2(next_reference_point.y - reference_point.y, next_reference_point.x - reference_point.x);
            break;
        }
    }
    return reference_heading;
}

math::Point2d
MultiAgentPID::compute_center_lane_reference_point(const double distance, map::Route& route)
{
    math::Point2d reference_point;
    for (int i = 0; i < route.center_lane.size(); i++)
    {
        if (route.center_lane[i].s > distance)
        {
            reference_point.x = route.center_lane[i].x;
            reference_point.y = route.center_lane[i].y;
            break;
        }
    }
    return reference_point;
}

} // namespace planner
} // namespace adore
