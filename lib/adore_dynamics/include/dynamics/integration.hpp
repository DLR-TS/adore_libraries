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
#pragma once
#include "adore_math/angles.h"

#include "dynamics/vehicle_command.hpp"

namespace adore
{
namespace dynamics
{
template<typename VehicleState>
VehicleState
euler_step( const VehicleState& state, const VehicleCommand& command, double dt, double wheelbase )
{
  // Create a copy of the current state to update
  VehicleState next_state = state;

  // Extract state variables
  const double& x   = state.x;
  const double& y   = state.y;
  const double& yaw = state.yaw_angle;
  const double& v   = state.vx;

  // Extract control inputs
  const double& delta = command.steering_angle;
  const double& a     = command.acceleration;

  // Compute derivatives using the bicycle model
  double cos_yaw   = std::cos( yaw + 0.5 * delta );
  double sin_yaw   = std::sin( yaw + 0.5 * delta );
  double tan_delta = std::tan( delta );

  double dx   = v * cos_yaw;
  double dy   = v * sin_yaw;
  double dyaw = ( v / wheelbase ) * tan_delta;
  double dv   = a;

  // Update state variables using Euler integration
  next_state.x         += dx * dt;
  next_state.y         += dy * dt;
  next_state.yaw_angle += dyaw * dt;
  next_state.vx        += dv * dt;
  next_state.time      += dt;

  // Normalize yaw angle to [-pi, pi]
  next_state.yaw_angle = adore::math::normalize_angle( next_state.yaw_angle );

  // Update other state variables if necessary
  // For example, vy, yaw_rate, etc., can be set to zero or computed if needed
  next_state.vy       = 0.0;
  next_state.yaw_rate = dyaw;

  return next_state;
}

// FOR USING RK4 model implementation...
template<typename VehicleState>
Eigen::Matrix<double, 10, 1>
vehicle_state_to_eigen( const VehicleState& vehicle_state, double acceleration, double steering_angle )
{
  Eigen::Matrix<double, 10, 1> state_matrix;

  state_matrix( 0, 0 ) = vehicle_state.x;
  state_matrix( 1, 0 ) = vehicle_state.y;
  state_matrix( 2, 0 ) = vehicle_state.yaw_angle;
  state_matrix( 3, 0 ) = vehicle_state.vx;
  state_matrix( 4, 0 ) = vehicle_state.vy;
  state_matrix( 5, 0 ) = vehicle_state.yaw_rate;
  state_matrix( 6, 0 ) = acceleration;
  state_matrix( 7, 0 ) = steering_angle;
  state_matrix( 8, 0 ) = 0; // Placeholder for additional state values
  state_matrix( 9, 0 ) = 0; // Placeholder for additional state values

  return state_matrix;
}

template<typename VehicleState>
void
eigen_to_vehicle_state( const Eigen::Matrix<double, 10, 1>& state_matrix, VehicleState& vehicle_state )
{
  vehicle_state.x              = state_matrix( 0, 0 );
  vehicle_state.y              = state_matrix( 1, 0 );
  vehicle_state.z              = 0; // Assuming 2D, so z is set to 0
  vehicle_state.yaw_angle      = state_matrix( 2, 0 );
  vehicle_state.vx             = state_matrix( 3, 0 );
  vehicle_state.vy             = state_matrix( 4, 0 );
  vehicle_state.yaw_rate       = state_matrix( 5, 0 );
  vehicle_state.ax             = state_matrix( 6, 0 );
  vehicle_state.steering_angle = state_matrix( 7, 0 );
}

template<typename VehicleState, typename VehicleModel>
VehicleState
rk4_step( const VehicleState& state, const VehicleCommand& command, double time, double dt, const VehicleModel& model )
{

  VehicleState next_state = state;
  // Convert the current vehicle state and control inputs into an Eigen matrix
  Eigen::Matrix<double, 10, 1> last_update_state = vehicle_state_to_eigen( state, command.acceleration, command.steering_angle );

  // Calculate time steps between the last and current updates
  std::vector<double> time_steps_between_updates = sequence( 0, dt, time );

  // Solve for the next vehicle state using the ODE RK4 solver
  Eigen::Matrix<double, 10, 1> updated_state = solve_for_next_state_with_ode_rk4( model, time_steps_between_updates, last_update_state );

  // Convert the Eigen matrix back into the vehicle state struct
  eigen_to_vehicle_state( updated_state, next_state );

  // Compute the steering rate as the difference between the current and previous steering angles
  next_state.steering_rate = ( state.steering_angle - next_state.steering_angle ) / time;

  next_state.time = state.time + time;

  return next_state;
}


} // namespace dynamics
} // namespace adore
