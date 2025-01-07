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

  void                 set_parameters( const std::map<std::string, double>& params );
  dynamics::TrafficParticipantSet plan_trajectories( const dynamics::TrafficParticipantSet& traffic_participant_set, 
                                        const dynamics::VehicleCommandLimits& limits );

  double       desired_acceleration     = 0.3;
  double       desired_deceleration     = 0.3;
  double       max_lateral_acceleration = 0.5;
  int          number_integration_nodes = 120;
  double       dt                       = 0.05;
  const double min_point_distance       = 0.05;
  double       max_speed                = 2.5;
  double       wheelbase                = 2.7;


private:
  double compute_error_direction(const double x_position, const double y_position, const double heading, 
                                const double x_objective, const double y_objective);
  double compute_error_distance(const double x_position, const double y_position, const double heading, 
                                const double x_objective, const double y_objective);
  double compute_error_speed(const double speed);
  double compute_error_heading(const double heading, const double heading_objective);
  math::Point2d compute_center_lane_reference_point(const double distance, map::Route& route);
  double compute_center_lane_reference_heading(const double distance, map::Route& route);
};
} // namespace planner
} // namespace adore
