#pragma once

#include <cmath>

#include <algorithm> // for std::clamp

#include "dynamics/physical_vehicle_parameters.hpp"
#include "dynamics/vehicle_command.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace dynamics
{


static inline VehicleStateDynamic
dynamic_bicycle_model( const VehicleStateDynamic& state, const PhysicalVehicleParameters& params, const VehicleCommand& cmd )
{
  VehicleStateDynamic ds; // derivative of state

  // Unpack state.
  double psi   = state.yaw_angle;
  double vx    = state.vx;
  double vy    = state.vy;
  double omega = state.yaw_rate;

  // Use the commanded acceleration and steering angle.
  double a_cmd = cmd.acceleration;
  double delta = cmd.steering_angle;

  // Check for near-zero vx to avoid division by zero.
  constexpr double epsilon = 1e-6;
  if( std::abs( vx ) < epsilon )
  {
    // When there's no longitudinal velocity, update only what makes sense.
    ds.x              = 0.0;
    ds.y              = 0.0;
    ds.z              = 0.0;
    ds.vx             = a_cmd; // We can still integrate acceleration.
    ds.vy             = 0.0;
    ds.yaw_angle      = state.yaw_angle;
    ds.yaw_rate       = 0.0;
    ds.steering_angle = state.steering_rate;
    ds.steering_rate  = 0.0;
    ds.ax             = 0.0;
    ds.ay             = 0.0;
    ds.time           = 1.0;
    return ds;
  }

  // For now, lambda is 0 (as in your original code).
  double lambda = 0.0;
  // Effective lateral velocity.
  double vyr = vy - lambda * state.yaw_rate;

  // Compute tire force contributions.
  // Note: Make sure that params.wheelbase is nonzero.
  double fyf_fzf = -params.friction_coefficient * ( params.rear_axle_to_cog / params.wheelbase ) * params.gravity
                 * params.front_tire_stiffness * ( ( ( vyr ) + params.wheelbase * omega ) / vx - delta );
  double fyr_fzr = -params.friction_coefficient * ( params.cog_to_front_axle / params.wheelbase ) * params.gravity
                 * params.rear_tire_stiffness * ( vyr / vx );

  double ay_dynamic = fyf_fzf + fyr_fzr;
  double domega = ( 1.0 / params.rotational_inertia_div_mass ) * ( params.cog_to_front_axle * fyf_fzf - params.rear_axle_to_cog * fyr_fzr );

  // Compute state derivatives.
  ds.x              = std::cos( psi ) * vx - std::sin( psi ) * vy;
  ds.y              = 0.0; // Lateral position update may be omitted in the dynamic branch.
  ds.z              = 0.0;
  ds.vx             = a_cmd;
  ds.vy             = ay_dynamic;
  ds.yaw_angle      = omega;
  ds.yaw_rate       = domega;
  ds.steering_angle = state.steering_angle;
  ds.steering_rate  = 0.0; // No higher order steering dynamics modelled.
  ds.ax             = 0.0;
  ds.ay             = 0.0;
  ds.time           = 1.0; // time derivative is 1.
  return ds;
}

//
// Kinematic Bicycle params
// This params uses geometric relationships to compute the yaw rate and positions.
// It is typically used at lower speeds when dynamics effects are less pronounced.
//
static inline VehicleStateDynamic
kinematic_bicycle_model( const VehicleStateDynamic& state, const PhysicalVehicleParameters& params, const VehicleCommand& cmd )
{
  VehicleStateDynamic ds;

  // Unpack state.
  double psi = state.yaw_angle;
  double vx  = state.vx;
  double vy  = state.vy;

  // Use commanded acceleration.
  double a_cmd = cmd.acceleration;
  double delta = cmd.steering_angle;

  // Derivatives from kinematic relationships.
  ds.x         = std::cos( psi ) * vx;
  ds.y         = std::sin( psi ) * vx;
  ds.z         = 0.0;
  ds.vx        = a_cmd;
  ds.vy        = 0.0; // Lateral acceleration is not paramsed explicitly.
  ds.yaw_angle = vx * std::tan( delta ) / params.wheelbase;
  ds.yaw_rate  = 0.0;

  // Simple first-order steering dynamics:
  ds.steering_angle = state.steering_rate; // derivative of steering angle is current rate
  ds.steering_rate  = 0.0;                 // Not modelled

  ds.ax   = 0.0;
  ds.ay   = 0.0;
  ds.time = 1.0;
  return ds;
}
} // namespace dynamics
} // namespace adore