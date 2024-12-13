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
#include "dynamics/OdeRK4.hpp"

namespace adore
{
namespace dynamics
{
std::vector<double>
sequence( double time_of_last_update, double integration_size, double time_current )
{
  int                 n = ceil( ( time_current - time_of_last_update ) / integration_size ) + 1;
  std::vector<double> time_steps_array;
  for( int i = 0; i < n; i++ )
  {
    time_steps_array.push_back( time_of_last_update + integration_size * static_cast<double>( i ) );
  }
  time_steps_array.push_back( time_current );
  return time_steps_array;
}

Eigen::Matrix<double, 10, 1>
derive_10d_state_vector( double time, PhysicalVehicleParameters model, Eigen::Matrix<double, 10, 1> vector_state_to_derive )
{
  Eigen::Matrix<double, 10, 1> derived_vector_to_return;

  double lambda = 0.0; // Placeholder for unknown variable

  double psi = vector_state_to_derive( 2, 0 );
  double vx  = vector_state_to_derive( 3, 0 );
  double vyr = vector_state_to_derive( 4, 0 ) - lambda * vector_state_to_derive( 5, 0 );

  double vy    = vector_state_to_derive( 4, 0 );
  double omega = vector_state_to_derive( 5, 0 );
  double ax    = vector_state_to_derive( 6, 0 );
  double delta = vector_state_to_derive( 7, 0 ); // steering wheel angle

  if( delta < model.steering_angle_min )
  {
    delta = model.steering_angle_min;
  }

  if( delta > model.steering_angle_max )
  {
    delta = model.steering_angle_max;
  }

  double dax    = vector_state_to_derive( 8, 0 ); // Derivative
  double ddelta = vector_state_to_derive( 9, 0 ); // Derivative

  // assuming the car is in the drive state
  if( vx < 0.0 && ax < 0.0 )
  {
    ax = -ax;
  }

  bool false_bool = false;
  if( vx > 1.0 && false_bool == true ) // if there is any forward velocity
  {
    double fyf_fzf, fyr_fzr;

    fyf_fzf = -model.friction_coefficient * ( model.rear_axle_to_cog / model.vehicle_length ) * model.gravity * model.front_tire_stiffness
            * ( ( vyr + model.vehicle_length * omega ) / vx - delta );
    fyr_fzr = -model.friction_coefficient * ( model.cog_to_front_axle / model.vehicle_length ) * model.gravity * model.rear_tire_stiffness
            * ( vyr / vx );

    double ay     = fyf_fzf + fyr_fzr;
    double domega = ( 1.0 / model.rotational_inertia_div_mass ) * ( model.cog_to_front_axle * fyf_fzf - model.rear_axle_to_cog * fyr_fzr );

    derived_vector_to_return( 0, 0 ) = cos( psi ) * vx - sin( psi ) * vy;
    derived_vector_to_return( 1, 0 ) = 0;
    derived_vector_to_return( 2, 0 ) = omega;
    derived_vector_to_return( 3, 0 ) = ax;
    derived_vector_to_return( 4, 0 ) = 0;
    derived_vector_to_return( 5, 0 ) = domega;
    derived_vector_to_return( 6, 0 ) = dax;
    derived_vector_to_return( 7, 0 ) = ddelta;
    derived_vector_to_return( 8, 0 ) = 0;
    derived_vector_to_return( 9, 0 ) = 0;

    return derived_vector_to_return;
  }

  // Derive kinematic (only if dynamic is unavailable)
  double kappa                     = tan( delta ) / model.vehicle_length;
  derived_vector_to_return( 0, 0 ) = cos( psi ) * vx - sin( psi ) * vy;
  derived_vector_to_return( 1, 0 ) = sin( psi ) * vx + cos( psi ) * vy;
  derived_vector_to_return( 2, 0 ) = vx * kappa;
  derived_vector_to_return( 3, 0 ) = ax;
  derived_vector_to_return( 4, 0 ) = -5.0 * ( vy - vx * kappa * lambda );
  derived_vector_to_return( 5, 0 ) = -5.0 * ( omega - vx * kappa );
  derived_vector_to_return( 6, 0 ) = dax;
  derived_vector_to_return( 7, 0 ) = ddelta;
  derived_vector_to_return( 8, 0 ) = 0;
  derived_vector_to_return( 9, 0 ) = 0;

  return derived_vector_to_return;
}

Eigen::Matrix<double, 10, 1>
solve_for_next_state_with_ode_rk4( PhysicalVehicleParameters model, std::vector<double> time_steps_array,
                                   Eigen::Matrix<double, 10, 1> ten_d_state )
{
  int             n = 10;
  int             m = time_steps_array.size();
  Eigen::MatrixXd results( n, m );

  Eigen::MatrixXd k1( n, 1 );
  Eigen::MatrixXd k2( n, 1 );
  Eigen::MatrixXd k3( n, 1 );
  Eigen::MatrixXd k4( n, 1 );

  Eigen::Matrix<double, 10, 1> state = ten_d_state;

  for( int i = 1; i < m; i++ )
  {
    double dt        = time_steps_array[i] - time_steps_array[i - 1];
    k1               = derive_10d_state_vector( time_steps_array[i - 1], model, state );
    k2               = derive_10d_state_vector( time_steps_array[i - 1] + dt / 2, model, state + ( dt / 2 ) * k1 );
    k3               = derive_10d_state_vector( time_steps_array[i - 1] + dt / 2, model, state + ( dt / 2 ) * k2 );
    k4               = derive_10d_state_vector( time_steps_array[i - 1] + dt, model, state + dt * k3 );
    results.col( i ) = state + ( dt / 6 ) * ( k1 + 2 * k2 + 2 * k3 + k4 );

    results( 0, 0 ) = 0;

    state = results.col( i );
  }

  return state;
}
} // namespace dynamics
} // namespace adore
