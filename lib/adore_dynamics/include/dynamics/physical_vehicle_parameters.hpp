/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Mikkel Skov Maarssø
 ********************************************************************************/
#pragma once

#include <string>

namespace adore
{
namespace dynamics
{


class PhysicalVehicleParameters
{
public:

  std::string vehicle_id                         = "missing";
  double      cog_to_front_axle                  = 0;
  double      rear_axle_to_cog                   = 0;
  double      vehicle_length                     = 0;
  double      front_axle_to_front_border         = 0;
  double      rear_border_to_rear_axle           = 0;
  double      mass                               = 0;
  double      friction_coefficient               = 0;
  double      gravity                            = 0;
  double      cog_height                         = 0;
  double      front_tire_stiffness               = 0;
  double      rear_tire_stiffness                = 0;
  double      rotational_inertia_div_mass        = 0;
  double      front_track_width                  = 0;
  double      rear_track_width                   = 0;
  double      body_width                         = 0;
  double      steering_ratio                     = 0;
  double      steering_angle_offset_measured     = 0;
  double      steering_angle_offset_command      = 0;
  double      steering_angle_max                 = 0;
  double      steering_angle_min                 = 0;
  double      cornering_stiffness                = 0;
  double      brake_balance_front                = 0;
  double      acceleration_balance_front         = 0;
  double      observation_point_for_position     = 0;
  double      observation_point_for_velocity     = 0;
  double      observation_point_for_acceleration = 0;
  double      vehicle_flag                       = 0;

  PhysicalVehicleParameters() {};

  PhysicalVehicleParameters( std::string vehicle_name )
  {
    if( vehicle_name == "bicycle" )
    {
      get_bicycle_vehicle_parameters();
      return;
    }
  }

  void
  get_bicycle_vehicle_parameters()
  {
    vehicle_id                         = "bicycle";
    cog_to_front_axle                  = 1.014;
    rear_axle_to_cog                   = 1.676;
    vehicle_length                     = cog_to_front_axle + rear_axle_to_cog;
    front_axle_to_front_border         = 0.97;
    rear_border_to_rear_axle           = 1.12;
    mass                               = 1800;
    friction_coefficient               = 0.8;
    gravity                            = 9.81;
    cog_height                         = 0.5;
    front_tire_stiffness               = 10.8;
    rear_tire_stiffness                = 17.8;
    rotational_inertia_div_mass        = 1.57;
    front_track_width                  = 1.7;
    rear_track_width                   = 1.7;
    body_width                         = 1.82;
    steering_ratio                     = 1.0;
    steering_angle_offset_measured     = 0.0;
    steering_angle_offset_command      = 0.0;
    steering_angle_max                 = 0.7;
    steering_angle_min                 = -0.7;
    cornering_stiffness                = 63000.0;
    brake_balance_front                = 0.6;
    acceleration_balance_front         = 0.4;
    observation_point_for_position     = 0.0;
    observation_point_for_velocity     = 0.0;
    observation_point_for_acceleration = 0.0;
    vehicle_flag                       = 0.0;
  }
};
} // namespace dynamics
} // namespace adore
