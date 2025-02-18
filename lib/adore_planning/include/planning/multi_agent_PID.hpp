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

#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

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

  double       desired_acceleration                = 1.0;
  double       desired_deceleration                = 1.0;
  double       max_lateral_acceleration            = 0.5;
  int          number_of_integration_steps         = 200;
  double       dt                                  = 0.05;
  const double min_point_distance                  = 0.05;
  double       wheelbase                           = 2.7;
  double       max_speed                           = 5.0;
  double       obstacle_avoidance_offset_threshold = 1.0;
  double       k_yaw                               = 2.0;
  double       k_distance                          = 2.0;
  double       k_speed                             = 0.5;
  double       k_goal_point                        = 10.0;
  double       k_repulsive_force                   = 100.0;
  double       k_obstacle_avoidance_longitudinal   = 3.0;
  double       k_obstacle_avoidance_lateral        = 3.0;


private:

  double compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose );
  double compute_error_yaw( const double current_yaw, const double yaw_objective );
  std::tuple<double, double, double> compute_distance_speed_offset_nearest_obstacle(
    dynamics::TrafficParticipantSet& traffic_participant_set, int id_vehicle );
  double                    compute_idm_velocity( double dist_to_nearest_object, double distance_to_goal, double nearest_object_speed,
                                                  const dynamics::VehicleStateDynamic& current_state );
  std::pair<double, double> compute_obstacle_avoidance_speed_component_errors( const dynamics::VehicleStateDynamic& current_state,
                                                                               const double                         current_s,
                                                                               dynamics::TrafficParticipantSet&     traffic_participant_set,
                                                                               const int                            vehicle_id );
  double                    sigmoid_activation( double d, double d_threshold, double k );
  std::pair<double, double> compute_target_speed_components( const dynamics::VehicleStateDynamic& current_state, const double current_s,
                                                             const dynamics::VehicleStateDynamic& other_participant_state,
                                                             map::Route&                          route );
};
} // namespace planner
} // namespace adore
