#pragma once

#include <string_view>

namespace adore
{
namespace dynamics
{

struct PhysicalVehicleParameters
{
  std::string_view vehicle_id;
  double           cog_to_front_axle;
  double           rear_axle_to_cog;
  double           vehicle_length;
  double           front_axle_to_front_border;
  double           rear_border_to_rear_axle;
  double           mass;
  double           friction_coefficient;
  double           gravity;
  double           cog_height;
  double           front_tire_stiffness;
  double           rear_tire_stiffness;
  double           rotational_inertia_div_mass;
  double           front_track_width;
  double           rear_track_width;
  double           body_width;
  double           steering_ratio;
  double           steering_angle_offset_measured;
  double           steering_angle_offset_command;
  double           steering_angle_max;
  double           steering_angle_min;
  double           cornering_stiffness;
  double           brake_balance_front;
  double           acceleration_balance_front;
  double           observation_point_for_position;
  double           observation_point_for_velocity;
  double           observation_point_for_acceleration;
  double           vehicle_flag;

  // A constexpr default constructor.
  constexpr PhysicalVehicleParameters() noexcept :
    vehicle_id( "missing" ),
    cog_to_front_axle( 0.0 ),
    rear_axle_to_cog( 0.0 ),
    vehicle_length( 0.0 ),
    front_axle_to_front_border( 0.0 ),
    rear_border_to_rear_axle( 0.0 ),
    mass( 0.0 ),
    friction_coefficient( 0.0 ),
    gravity( 0.0 ),
    cog_height( 0.0 ),
    front_tire_stiffness( 0.0 ),
    rear_tire_stiffness( 0.0 ),
    rotational_inertia_div_mass( 0.0 ),
    front_track_width( 0.0 ),
    rear_track_width( 0.0 ),
    body_width( 0.0 ),
    steering_ratio( 0.0 ),
    steering_angle_offset_measured( 0.0 ),
    steering_angle_offset_command( 0.0 ),
    steering_angle_max( 0.0 ),
    steering_angle_min( 0.0 ),
    cornering_stiffness( 0.0 ),
    brake_balance_front( 0.0 ),
    acceleration_balance_front( 0.0 ),
    observation_point_for_position( 0.0 ),
    observation_point_for_velocity( 0.0 ),
    observation_point_for_acceleration( 0.0 ),
    vehicle_flag( 0.0 )
  {}

  // A constexpr factory function for the bicycle parameters.
  static constexpr PhysicalVehicleParameters
  make_bicycle_parameters() noexcept
  {
    PhysicalVehicleParameters params{};
    params.vehicle_id                         = "bicycle";
    params.cog_to_front_axle                  = 1.014;
    params.rear_axle_to_cog                   = 1.676;
    params.vehicle_length                     = params.cog_to_front_axle + params.rear_axle_to_cog;
    params.front_axle_to_front_border         = 0.97;
    params.rear_border_to_rear_axle           = 1.12;
    params.mass                               = 1800;
    params.friction_coefficient               = 0.8;
    params.gravity                            = 9.81;
    params.cog_height                         = 0.5;
    params.front_tire_stiffness               = 10.8;
    params.rear_tire_stiffness                = 17.8;
    params.rotational_inertia_div_mass        = 1.57;
    params.front_track_width                  = 1.7;
    params.rear_track_width                   = 1.7;
    params.body_width                         = 1.82;
    params.steering_ratio                     = 1.0;
    params.steering_angle_offset_measured     = 0.0;
    params.steering_angle_offset_command      = 0.0;
    params.steering_angle_max                 = 0.7;
    params.steering_angle_min                 = -0.7;
    params.cornering_stiffness                = 63000.0;
    params.brake_balance_front                = 0.6;
    params.acceleration_balance_front         = 0.4;
    params.observation_point_for_position     = 0.0;
    params.observation_point_for_velocity     = 0.0;
    params.observation_point_for_acceleration = 0.0;
    params.vehicle_flag                       = 0.0;
    return params;
  }

  // A constexpr constructor that selects the appropriate parameters.
  constexpr PhysicalVehicleParameters( std::string_view vehicle_name ) noexcept :
    PhysicalVehicleParameters() // Delegate to the default constructor.
  {
    // In a constexpr context, comparisons of std::string_view with literals are allowed.
    if( vehicle_name == "bicycle" )
    {
      *this = make_bicycle_parameters();
    }
    // Otherwise, leave the default ("missing") parameters.
  }
};

} // namespace dynamics
} // namespace adore
