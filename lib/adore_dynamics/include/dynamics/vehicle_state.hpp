#pragma once

/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Mikkel Skov Maarssø
 *    Marko Mizdrak
 ********************************************************************************/
#include "adore_math/angles.h"

#include "OdeRK4.hpp"
#include "dynamics/integration.hpp"
#include "dynamics/physical_vehicle_parameters.hpp"
#include "dynamics/vehicle_command.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

namespace adore
{
namespace dynamics
{

enum GearState
{
  NEUTRAL,
  DRIVING,
  REVERSE,
  PARKING
};

struct VehicleInfo
{
  GearState gear_state;
  double    wheel_speed;
  bool      left_indicator_on;
  bool      right_indicator_on;
  bool      automatic_steering_on;
  bool      automatic_acceleration_on;
  bool      automatic_acceleration_active;
  bool      clearance;
};

struct VehicleStateSimple
{
  VehicleStateSimple() {}

  VehicleStateSimple( double x, double y, double yaw_angle, double vx ) :
    x( x ),
    y( y ),
    yaw_angle( yaw_angle ),
    vx( vx )
  {}

  double x         = 0.0;
  double y         = 0.0;
  double yaw_angle = 0.0;
  double vx        = 0.0;

  // Friend declaration for operator<<
  friend std::ostream& operator<<( std::ostream& os, const VehicleStateSimple& state );
};

// VehicleStateDynamic Struct
struct VehicleStateDynamic
{
  VehicleStateDynamic() {}

  VehicleStateDynamic( double x, double y, double z, double vx, double vy, double yaw_angle, double yaw_rate, double ax, double ay ) :
    x( x ),
    y( y ),
    z( z ),
    vx( vx ),
    vy( vy ),
    yaw_angle( yaw_angle ),
    yaw_rate( yaw_rate ),
    ax( ax ),
    ay( ay )
  {}

  void integrate_up_to_time( double time );

  // Member function to convert to VehicleStateSimple
  VehicleStateSimple to_vehicle_state_simple() const;

  // Static member function to create from VehicleStateSimple
  static VehicleStateDynamic from_vehicle_state_simple( const VehicleStateSimple& simple_state );

  double x              = 0.0;
  double y              = 0.0;
  double z              = 0.0;
  double vx             = 0.0;
  double vy             = 0.0;
  double yaw_angle      = 0.0;
  double yaw_rate       = 0.0;
  double steering_angle = 0.0;
  double steering_rate  = 0.0;
  double ax             = 0.0;
  double ay             = 0.0;
  double time           = 0.0;

  // Friend declaration for operator<<
  friend std::ostream& operator<<( std::ostream& os, const VehicleStateDynamic& state );
};

VehicleStateDynamic interpolate_states_linear( const VehicleStateDynamic& state1, const VehicleStateDynamic& state2, double alpha );

} // namespace dynamics
} // namespace adore
