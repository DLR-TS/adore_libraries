#pragma once

#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace adore
{
namespace dynamics
{

struct PhysicalVehicleParameters
{
  double cog_to_front_axle;
  double rear_axle_to_cog;
  double front_axle_to_front_border;
  double rear_border_to_rear_axle;
  double mass;
  double friction_coefficient;
  double gravity;
  double cog_height;
  double front_tire_stiffness;
  double rear_tire_stiffness;
  double rotational_inertia_div_mass;
  double front_track_width;
  double rear_track_width;
  double body_width;
  double steering_ratio;
  double steering_angle_offset_measured;
  double steering_angle_offset_command;
  double steering_angle_max;
  double steering_angle_min;
  double cornering_stiffness;
  double brake_balance_front;
  double acceleration_balance_front;
  double observation_point_for_position;
  double observation_point_for_velocity;
  double observation_point_for_acceleration;
  double vehicle_flag;
  double wheelbase;

  // Constructor that loads parameters from a JSON file.
  PhysicalVehicleParameters( const std::string& file_path )
  {
    std::ifstream ifs( file_path );
    if( !ifs.is_open() )
    {
      throw std::runtime_error( "Could not open file: " + file_path );
    }
    nlohmann::json j;
    ifs >> j;

    cog_to_front_axle                  = j.at( "cog_to_front_axle" ).get<double>();
    rear_axle_to_cog                   = j.at( "rear_axle_to_cog" ).get<double>();
    wheelbase                          = j.at( "wheelbase" ).get<double>();
    front_axle_to_front_border         = j.at( "front_axle_to_front_border" ).get<double>();
    rear_border_to_rear_axle           = j.at( "rear_border_to_rear_axle" ).get<double>();
    mass                               = j.at( "mass" ).get<double>();
    friction_coefficient               = j.at( "friction_coefficient" ).get<double>();
    gravity                            = j.at( "gravity" ).get<double>();
    cog_height                         = j.at( "cog_height" ).get<double>();
    front_tire_stiffness               = j.at( "front_tire_stiffness" ).get<double>();
    rear_tire_stiffness                = j.at( "rear_tire_stiffness" ).get<double>();
    rotational_inertia_div_mass        = j.at( "rotational_inertia_div_mass" ).get<double>();
    front_track_width                  = j.at( "front_track_width" ).get<double>();
    rear_track_width                   = j.at( "rear_track_width" ).get<double>();
    body_width                         = j.at( "body_width" ).get<double>();
    steering_ratio                     = j.at( "steering_ratio" ).get<double>();
    steering_angle_offset_measured     = j.at( "steering_angle_offset_measured" ).get<double>();
    steering_angle_offset_command      = j.at( "steering_angle_offset_command" ).get<double>();
    steering_angle_max                 = j.at( "steering_angle_max" ).get<double>();
    steering_angle_min                 = j.at( "steering_angle_min" ).get<double>();
    cornering_stiffness                = j.at( "cornering_stiffness" ).get<double>();
    brake_balance_front                = j.at( "brake_balance_front" ).get<double>();
    acceleration_balance_front         = j.at( "acceleration_balance_front" ).get<double>();
    observation_point_for_position     = j.at( "observation_point_for_position" ).get<double>();
    observation_point_for_velocity     = j.at( "observation_point_for_velocity" ).get<double>();
    observation_point_for_acceleration = j.at( "observation_point_for_acceleration" ).get<double>();
    vehicle_flag                       = j.at( "vehicle_flag" ).get<double>();
    std::cerr << "LOADED VEHICLE PARAMETERS " << file_path << std::endl;
  }

  PhysicalVehicleParameters() {}; // default values
};

} // namespace dynamics
} // namespace adore
