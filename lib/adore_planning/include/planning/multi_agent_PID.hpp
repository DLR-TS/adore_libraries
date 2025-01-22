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
#pragma once

#include <optional>
#include <unordered_map>

#include "adore_map/map.hpp"

#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"
#include "dynamics/traffic_participant.hpp"

namespace adore
{
namespace planner
{
class MultiAgentPID
{
public:

  MultiAgentPID();

  void set_parameters( const std::map<std::string, double>& params );
  void plan_trajectories( dynamics::TrafficParticipantSet& traffic_participant_set, const dynamics::VehicleCommandLimits& limits );

  double       desired_acceleration     = 0.3;
  double       desired_deceleration     = 0.3;
  double       max_lateral_acceleration = 0.5;
  int          number_of_integration_steps = 200;
  double       dt                       = 0.05;
  const double min_point_distance       = 0.05;
  double       wheelbase                = 2.7;
  double       max_speed                = 5.0;
  double       k_direction              = 0.6;
  double       k_yaw                    = 0.1;
  double       k_distance               = 0.1;
  double       k_speed                  = 0.5;
  double       k_goal_point             = 1.0;
  double       k_repulsive_force        = 1.0;


private:
  double compute_error_direction( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose );
  double compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose );
  double compute_error_speed( const double speed );
  double compute_error_yaw( const double current_yaw, const double yaw_objective );
  double compute_distance_from_nearest_obstacle( dynamics::TrafficParticipantSet& traffic_participant_set, int id_vehicle );
};
} // namespace planner
} // namespace adore
