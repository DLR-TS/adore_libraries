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
 *    Sanath Himasekhar Konthala
 *    Marko Mizdrak
 ********************************************************************************/
#include "controllers/NMPC.hpp"

namespace adore
{
namespace controllers
{


void
NMPC::set_parameters( const std::map<std::string, double>& params )
{
  options.intermediateIntegration = 2;
  options.OptiNLC_ACC             = 1e-08;
  options.maxNumberOfIteration    = 500;
  options.OSQP_verbose            = false;
  options.OSQP_max_iter           = 400;
  options.OptiNLC_time_limit      = 0.04;

  for( const auto& [name, value] : params )
  {
    if( name == "intermediate_integration" )
      options.intermediateIntegration = static_cast<int>( value );
    if( name == "max_iterations" )
      options.maxNumberOfIteration = static_cast<int>( value );
    if( name == "osqp_max_iterations" )
      options.OSQP_max_iter = static_cast<int>( value );
    if( name == "OptiNLC_ACC" )
      options.OptiNLC_ACC = value;
    if( name == "time_limit" )
      options.OptiNLC_time_limit = value;
    if( name == "debug_active" )
    {
      options.OSQP_verbose = static_cast<bool>( value );
    }
  }
  options.timeStep = sim_time / control_points;
}

// Helper function to set constraints
void
NMPC::setup_constraints( OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points>& ocp )
{
  // Define a simple input update method
  ocp.setInputUpdate(
    [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input, double currentTime, void* userData ) {
      VECTOR<double, input_size> update_input = { input[DELTA], input[ACC] };
      return update_input;
    } );

  // State Constraints
  ocp.setUpdateStateLowerBounds( [&]( const VECTOR<double, NMPC::state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, NMPC::state_size> state_constraints;
    state_constraints.setConstant( -std::numeric_limits<double>::infinity() );
    return state_constraints;
  } );

  ocp.setUpdateStateUpperBounds( [&]( const VECTOR<double, NMPC::state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, NMPC::state_size> state_constraints;
    state_constraints.setConstant( std::numeric_limits<double>::infinity() );
    return state_constraints;
  } );

  // Input Constraints
  ocp.setUpdateInputLowerBounds( [&]( const VECTOR<double, NMPC::state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[0] = -limits.max_steering_angle; // Steering angle limit
    input_constraints[1] = limits.min_acceleration;    // Acceleration limit
    return input_constraints;
  } );

  ocp.setUpdateInputUpperBounds( [&]( const VECTOR<double, NMPC::state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[0] = limits.max_steering_angle; // Steering angle limit
    input_constraints[1] = limits.max_acceleration;   // Acceleration limit
    return input_constraints;
  } );

  // Define a functions constraints method
  ocp.setUpdateFunctionConstraints( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( 0.0 );
    return functions_constraint;
  } );

  ocp.setUpdateFunctionConstraintsLowerBounds( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( -std::numeric_limits<double>::infinity() );
    return functions_constraint;
  } );

  ocp.setUpdateFunctionConstraintsUpperBounds( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( std::numeric_limits<double>::infinity() );
    return functions_constraint;
  } );
}

void
NMPC::setup_objective_function( OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points>& ocp )
{
  ocp.setObjectiveFunction(
    [&]( const VECTOR<double, NMPC::state_size>& state, const VECTOR<double, input_size>& input, double current_time ) {
      return state[L]; // Minimize the cost function `L`
    } );
}

// Helper function to set up the solver and solve the problem
bool
NMPC::solve_mpc( OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points>& ocp,
                 VECTOR<double, state_size>& initial_state, VECTOR<double, input_size>& initial_input, std::vector<double>& delta_output,
                 std::vector<double>& acc_output, double current_time )
{

  OptiNLC_Solver<double, input_size, state_size, 0, control_points> solver( ocp );


  solver.solve( current_time, initial_state, initial_input );

  auto opt_x = solver.get_optimal_states();

  auto opt_u = solver.get_optimal_inputs();

  for( int i = 0; i < control_points; i++ )
  {
    delta_output.push_back( opt_u[i * input_size + 0] );
    acc_output.push_back( opt_u[i * input_size + 1] );
  }


  return solver.get_final_objective_function() <= 40.0;
}

// Public method to get the next vehicle command based on NMPC
dynamics::VehicleCommand
NMPC::get_next_vehicle_command( const dynamics::Trajectory& trajectory, const dynamics::VehicleStateDynamic& current_state )
{
  dynamics::VehicleCommand return_command;
  auto                     start_time = std::chrono::high_resolution_clock::now();

  // Initial state and input
  VECTOR<double, NMPC::input_size> initial_input = { current_state.steering_angle, current_state.ax };
  VECTOR<double, NMPC::state_size> initial_state = { current_state.x, current_state.y, current_state.yaw_angle, current_state.vx, 0.0 };

  // Create an MPC problem (OCP)
  OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points> ocp( &options );

  // Set up dynamic model, objective, and constraints
  setup_dynamic_model( ocp, trajectory );

  setup_objective_function( ocp );

  setup_constraints( ocp );

  // Solve the MPC problem
  std::vector<double> delta_output, acc_output;
  bool                success = solve_mpc( ocp, initial_state, initial_input, delta_output, acc_output, current_state.time );

  return_command.steering_angle = delta_output[0];
  return_command.acceleration   = acc_output[0];
  std::cerr << "opt out steer: " << delta_output[0] << "     acc: " << acc_output[0] << std::endl;

  // Handle bad conditions if the solution is not good
  if( !success )
  {}
  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  // Log cost, time taken, and convergence status
  std::cerr << "NMPC execution time: " << elapsed_seconds.count() << " seconds\n";

  return return_command;
}

void
NMPC::setup_dynamic_model( OptiNLC_OCP<double, NMPC::input_size, NMPC::state_size, 0, NMPC::control_points>& ocp,
                           const dynamics::Trajectory&                                                       trajectory )
{
  ocp.setDynamicModel( [&]( const VECTOR<double, NMPC::state_size>& state, const VECTOR<double, NMPC::input_size>& input,
                            VECTOR<double, NMPC::state_size>& derivative, double current_time, void* user_data ) {
    const double l = 2.69; // wheelbase, can be tuned based on your vehicle

    // Dynamic model equations
    derivative[X]   = state[V] * cos( state[PSI] );       // X derivative (velocity * cos(psi))
    derivative[Y]   = state[V] * sin( state[PSI] );       // Y derivative (velocity * sin(psi))
    derivative[PSI] = state[V] * tan( input[DELTA] ) / l; // PSI derivative (steering angle / wheelbase)
    derivative[V]   = input[ACC];                         // Velocity derivative (acceleration)

    // Interpolate trajectory points based on the current time
    auto reference_point = trajectory.get_state_at_time( current_time );

    const double lateral_error = -sin( state[PSI] ) * ( state[X] - reference_point.x )
                               + cos( state[PSI] ) * ( state[Y] - reference_point.y );

    derivative[L] = ( state[V] - reference_point.vx ) * ( state[V] - reference_point.vx )
                  + 2 * ( state[X] - reference_point.x ) * ( state[X] - reference_point.x ) + input[DELTA] * input[DELTA]
                  + 2 * ( state[Y] - reference_point.y ) * ( state[Y] - reference_point.y )
                  + 1 * atan2( sin( reference_point.yaw_angle - state[PSI] ), cos( reference_point.yaw_angle - state[PSI] ) )
                      * atan2( sin( reference_point.yaw_angle - state[PSI] ), cos( reference_point.yaw_angle - state[PSI] ) );
  } );
}

NMPC::NMPC()
{
  options.setDefaults();
}


} // namespace controllers
} // namespace adore
