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
 ********************************************************************************/
#include "planning/optinlc_trajectory_planner.hpp"

namespace adore
{
namespace planner
{

void
OptiNLCTrajectoryPlanner::set_parameters( const std::map<std::string, double>& params )
{
  options.intermediateIntegration = 3;
  options.OptiNLC_ACC             = 1e-4;
  options.maxNumberOfIteration    = 500;
  options.OSQP_verbose            = false;
  options.OSQP_max_iter           = 500;
  options.OptiNLC_time_limit      = 0.09;
  options.perturbation            = 1e-6;
  options.timeStep                = sim_time / control_points;

  for( const auto& [name, value] : params )
  {
    if( name == "wheel_base" )
      wheelbase = value;
    if( name == "lateral_weight" )
      lateral_weight = value;
    if( name == "heading_weight" )
      heading_weight = value;
    if( name == "maximum_velocity" )
      maximum_velocity = value;
    if( name == "min_distance_to_vehicle_ahead" )
      min_distance_to_vehicle_ahead = value;
    if( name == "look_ahead_for_curvature" )
      look_ahead_for_curvature = value;
    if( name == "look_behind_for_curvature" )
      look_behind_for_curvature = value;
  }
}

void
OptiNLCTrajectoryPlanner::setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{

  // Define a simple input update method
  ocp.setInputUpdate(
    [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input, double currentTime, void* userData ) {
      VECTOR<double, input_size> update_input = { input[DELTA] };
      return update_input;
    } );

  // State Constraints
  ocp.setUpdateStateLowerBounds( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, state_size> state_constraints;
    state_constraints.setConstant( -std::numeric_limits<double>::infinity() );
    state_constraints[V] = max_reverse_speed;
    return state_constraints;
  } );

  ocp.setUpdateStateUpperBounds( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, state_size> state_constraints;
    state_constraints.setConstant( std::numeric_limits<double>::infinity() );
    state_constraints[V] = max_forward_speed;
    return state_constraints;
  } );

  // Input Constraints
  ocp.setUpdateInputLowerBounds( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[DELTA] = -limits.max_steering_angle;
    return input_constraints;
  } );

  ocp.setUpdateInputUpperBounds( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[DELTA] = limits.max_steering_angle;
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
OptiNLCTrajectoryPlanner::setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setObjectiveFunction( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input, double current_time ) {
    return state[L]; // Minimize the cost function `L`
  } );
}

// Public method to get the next vehicle command based on OptiNLCTrajectoryPlanner
dynamics::Trajectory
OptiNLCTrajectoryPlanner::plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                           const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants )
{
  route_to_piecewise_polynomial reference_route = setup_optimizer_parameters_using_route( latest_route );
  auto                          start_time      = std::chrono::high_resolution_clock::now();

  // Initial state and input
  VECTOR<double, input_size> initial_input = { current_state.steering_angle };
  VECTOR<double, state_size> initial_state = { current_state.x, current_state.y, current_state.yaw_angle, current_state.vx, 0.0, 0.0 };

  // Create an MPC problem (OCP)
  OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points> ocp( &options );

  // Set up reference route
  setup_reference_route( reference_route );
  if( route_x.breaks.size() < 1 )
  {
    dynamics::Trajectory empty_trajectory;
    std::cerr << "end of route or invalid route received" << std::endl;
    return empty_trajectory;
  }

  // Set up reference velocity
  setup_reference_velocity( latest_route, current_state, latest_map, traffic_participants );

  // Set up dynamic model, objective, and constraints
  setup_dynamic_model( ocp );

  setup_objective_function( ocp );

  setup_constraints( ocp );

  // Solve the MPC problem
  OptiNLC_Solver<double, input_size, state_size, constraints_size, control_points> solver( ocp );

  solver.solve( current_state.time, initial_state, initial_input );

  auto   opt_x                   = solver.get_optimal_states();
  auto   opt_u                   = solver.get_optimal_inputs();
  auto   time                    = solver.getTime();
  double last_objective_function = solver.get_final_objective_function();

  if( bad_counter > 4 )
  {
    bad_counter = 0;
  }
  for( int i = 0; i < control_points / 2; i++ )
  {
    if( last_objective_function > 20.0 || opt_x[i * state_size + V] > 14.5 || opt_x[i * state_size + V] < 0.0 )
    {
      bad_condition  = true;
      bad_counter   += 1;
      break;
    }
  }

  dynamics::Trajectory planned_trajectory;
  for( int i = 0; i < control_points; i++ )
  {
    dynamics::VehicleStateDynamic state;
    state.x              = opt_x[i * state_size + X];
    state.y              = opt_x[i * state_size + Y];
    state.yaw_angle      = opt_x[i * state_size + PSI];
    state.vx             = opt_x[i * state_size + V];
    state.steering_angle = opt_u[i * input_size + DELTA];
    state.time           = time[i];
    if( i < control_points - 1 )
    {
      state.yaw_rate      = ( opt_x[( i + 1 ) * state_size + PSI] - opt_x[i * state_size + PSI] ) / options.timeStep;
      state.ax            = ( opt_x[( i + 1 ) * state_size + V] - opt_x[i * state_size + V] ) / options.timeStep;
      state.steering_rate = ( opt_u[( i + 1 ) * input_size + DELTA] - opt_u[i * input_size + DELTA] ) / options.timeStep;
    }
    planned_trajectory.states.push_back( state );
  }
  planned_trajectory.states[control_points - 1].yaw_rate      = planned_trajectory.states[control_points - 2].yaw_rate;
  planned_trajectory.states[control_points - 1].ax            = planned_trajectory.states[control_points - 2].ax;
  planned_trajectory.states[control_points - 1].steering_rate = planned_trajectory.states[control_points - 2].steering_rate;

  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  // Log cost, time taken, and convergence status
  if( bad_condition == false && bad_counter < 5 )
  {
    previous_trajectory = planned_trajectory;
    bad_counter         = 0;
    return planned_trajectory;
  }
  else
  {
    return previous_trajectory;
  }
}

void
OptiNLCTrajectoryPlanner::setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setDynamicModel( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input,
                            VECTOR<double, state_size>& derivative, double current_time, void* user_data ) {
    const double tau = 1.0; // Higher value means slower acceleration

    // Dynamic model equations
    derivative[X]   = state[V] * cos( state[PSI] );                      // X derivative (velocity * cos(psi))
    derivative[Y]   = state[V] * sin( state[PSI] );                      // Y derivative (velocity * sin(psi))
    derivative[PSI] = state[V] * tan( input[DELTA] ) / wheelbase;        // PSI derivative (steering angle / wheelbase)
    derivative[V]   = ( 1.0 / tau ) * ( reference_velocity - state[V] ); // Velocity derivative (first order)
    derivative[S]   = state[V];                                          // Progress derivate (velocity)

    // Reference trajectory point at current progress
    int    index             = pp.findIndex( state[S], route_x );
    double reference_x       = pp.splineEvaluation( index, state[S], route_x );
    double reference_y       = pp.splineEvaluation( index, state[S], route_y );
    double reference_heading = pp.splineEvaluation( index, state[S], route_heading );

    // Position error terms
    double dx = state[X] - reference_x;
    double dy = state[Y] - reference_y;

    // Calculate longitudinal and lateral errors relative to the vehicle's heading
    double cos_yaw = cos( reference_heading );
    double sin_yaw = sin( reference_heading );

    double lateral_cost  = -dx * sin_yaw + dy * cos_yaw;
    lateral_cost        *= lateral_cost * lateral_weight;

    // Heading error term
    double heading_cost  = atan2( -sin_yaw * cos( state[PSI] ) + cos_yaw * sin( state[PSI] ),
                                  cos_yaw * cos( state[PSI] ) + sin_yaw * sin( state[PSI] ) );
    heading_cost        *= heading_cost * heading_weight;

    // Steering input cost
    double steering_cost = input[DELTA] * input[DELTA] * steering_weight;

    // Total cost derivative
    derivative[L] = lateral_cost + heading_cost;
  } );
}

void
OptiNLCTrajectoryPlanner::setup_reference_route( route_to_piecewise_polynomial& reference_route )
{
  route_x       = reference_route.x;
  route_y       = reference_route.y;
  route_heading = reference_route.heading;
}

OptiNLCTrajectoryPlanner::OptiNLCTrajectoryPlanner()
{
  options.setDefaults();
  set_parameters( {} );
}

route_to_piecewise_polynomial
OptiNLCTrajectoryPlanner::setup_optimizer_parameters_using_route( const adore::map::Route& latest_route )
{
  auto start_time  = std::chrono::high_resolution_clock::now();
  distance_to_goal = latest_route.get_remaining_route_length();

  route_to_piecewise_polynomial route;
  double                        sim_time = 3.0; // 3.0 seconds road ahead with 50km/h speed

  double maximum_required_road_length = sim_time * ( 13.6 + 1. ); // 13.6 maximum urban velocity, 1. extension window for end of horizon, if
                                                                  // starting point of optimization was bad
  if( maximum_required_road_length < 0.10 )
  {
    return route;
  }

  if( latest_route.center_lane.size() < 1 )
  {
    return route;
  }

  int N = 0;
  for( int i = 0; i < latest_route.center_lane.size(); i++ )
  {
    N++;
    if( latest_route.center_lane[i].s > maximum_required_road_length )
    {
      break;
    }
  }

  route_to_follow.s.clear();
  route_to_follow.x.clear();
  route_to_follow.y.clear();
  route_to_follow.psi.clear();

  std::vector<double> w;

  double previous_s = 0.0;
  for( int i = 0; i < N; i++ )
  {
    if( latest_route.center_lane[i].s - previous_s > 0.75 )
    {
      route_to_follow.s.push_back( latest_route.center_lane[i].s );
      route_to_follow.x.push_back( latest_route.center_lane[i].x );
      route_to_follow.y.push_back( latest_route.center_lane[i].y );
      w.push_back( 1.0 );
      previous_s = latest_route.center_lane[i].s;
    }
  }

  if( route_to_follow.s.size() < 3 )
  {
    return route;
  }
  route_to_follow.s[0] = 0.0; // overwriting the first element to 0 (start from ego vehicle)
  route.x              = pp.CubicSplineSmoother( route_to_follow.s, route_to_follow.x, w, 0.9 );
  route.y              = pp.CubicSplineSmoother( route_to_follow.s, route_to_follow.y, w, 0.9 );

  std::vector<double> x, dx;
  std::vector<double> y, dy;
  pp.CubicSplineEvaluation( x, dx, route_to_follow.s, route.x );
  pp.CubicSplineEvaluation( y, dy, route_to_follow.s, route.y );
  for( int i = 0; i < route_to_follow.s.size() - 1; i++ )
  {
    if( dx[i] == 0.0 || dx.size() < 1 || dy.size() < 1 )
    {
      return route;
    }
    route_to_follow.psi.push_back( std::atan2( dy[i], dx[i] ) );
  }
  route_to_follow.psi[route_to_follow.s.size() - 1] = route_to_follow.psi[route_to_follow.s.size() - 2];
  route.heading                                     = pp.CubicSplineSmoother( route_to_follow.s, route_to_follow.psi, w, 0.75 );

  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  // Log cost, time taken, and convergence status

  return route;
}

void
OptiNLCTrajectoryPlanner::setup_reference_velocity( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                                    const map::Map&                        latest_map,
                                                    const dynamics::TrafficParticipantSet& traffic_participants )
{
  std::vector<adore::math::Point2d> path_for_curvature;
  for( int i = 0; i < look_ahead_for_curvature; i++ )
  {
    adore::math::Point2d point_for_curvature;
    int                  index = pp.findIndex( i, route_x );
    point_for_curvature.x      = pp.splineEvaluation( index, i, route_x );
    point_for_curvature.y      = pp.splineEvaluation( index, i, route_y );
    path_for_curvature.push_back( point_for_curvature );
  }

  std::vector<double> curvature  = calculate_curvature( path_for_curvature );
  distance_moved                += current_state.vx * 0.1;
  if( distance_moved > distance_to_add_behind )
  {
    curvature_behind.push_back( curvature[0] );
    distance_to_add_behind += 1;
  }

  if( curvature_behind.size() > look_behind_for_curvature ) // 10 meters look behind path
  {
    curvature_behind.erase( curvature_behind.begin() );
  }
  std::vector<double> total_curvature = curvature_behind;

  total_curvature.insert( total_curvature.end(), curvature.begin(), curvature.end() );

  double max_curvature = *std::max_element( total_curvature.begin(), total_curvature.end() );
  reference_velocity   = maximum_velocity / ( 1 + curvature_weight * max_curvature );

  double idm_velocity = calculate_idm_velocity( latest_route, current_state, latest_map, traffic_participants );
  reference_velocity  = std::min( reference_velocity, idm_velocity );

  // dynamic reference velocity adjusting based on the error the reference and current velocity
  reference_velocity = reference_velocity + velocity_error_gain * ( reference_velocity - current_state.vx );

  auto current_route_point_max_speed = latest_route.center_lane.front().max_speed;
  if( current_route_point_max_speed.has_value() )
  {
    reference_velocity = std::min( reference_velocity, current_route_point_max_speed.value() );
  }
}

double
OptiNLCTrajectoryPlanner::calculate_idm_velocity( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                                  const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants )
{
  double distance_to_object_min     = std::numeric_limits<double>::max();
  double distance_to_maintain_ahead = min_distance_to_vehicle_ahead;
  double idm_velocity               = maximum_velocity;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    math::Point2d object_position;
    object_position.x                      = participant.state.x;
    object_position.y                      = participant.state.y;
    auto [within_lane, distance_to_object] = latest_route.get_distance_along_route( latest_map, object_position );

    if( within_lane && distance_to_object < distance_to_object_min && distance_to_object > 0.2 )
    {
      distance_to_object_min = distance_to_object;
    }
  }

  double distance_for_idm = std::min( distance_to_object_min, distance_to_goal );

  if( distance_to_goal < distance_to_object_min && distance_to_goal < near_goal_distance )
  {
    distance_to_maintain_ahead = wheelbase / 2;
  }

  double s_star = distance_to_maintain_ahead + current_state.vx * desired_time_headway
                + current_state.vx * ( current_state.vx - front_vehicle_velocity ) / ( 2 * sqrt( max_acceleration * max_deceleration ) );
  idm_velocity = current_state.vx
               + max_acceleration
                   * ( 1
                       - ( current_state.vx / maximum_velocity ) * ( current_state.vx / maximum_velocity )
                           * ( current_state.vx / maximum_velocity ) * ( current_state.vx / maximum_velocity )
                       - ( s_star / distance_for_idm ) * ( s_star / distance_for_idm ) );
  if( idm_velocity < 0.0 )
  {
    idm_velocity = 0.0;
  }
  if( idm_velocity > maximum_velocity )
  {
    idm_velocity = maximum_velocity;
  }
  return idm_velocity;
}

std::vector<double>
OptiNLCTrajectoryPlanner::calculate_curvature( const std::vector<adore::math::Point2d>& path )
{
  int                 n = path.size();
  std::vector<double> curvature( n, 0.0 );

  // Check if there are enough points to calculate curvature
  if( n < 3 )
  {
    return curvature;
  }

  for( int i = 1; i < n - 1; ++i )
  {
    // Discrete first derivate
    double x_prime = ( path[i + 1].x - path[i - 1].x ) / 2.0;
    double y_prime = ( path[i + 1].y - path[i - 1].y ) / 2.0;

    // Discrete second derivative
    double x_double_prime = path[i + 1].x - 2 * path[i].x + path[i - 1].x;
    double y_double_prime = path[i + 1].y - 2 * path[i].y + path[i - 1].y;

    double numerator   = std::abs( x_prime * y_double_prime - y_prime * x_double_prime );
    double denominator = std::pow( x_prime * x_prime + y_prime * y_prime, 1.5 );

    if( denominator != 0 )
    {
      curvature[i] = numerator / denominator;
    }
    else
    {
      curvature[i] = 0.0;
    }
  }

  curvature[0]     = curvature[1];
  curvature[n - 1] = curvature[n - 2];

  return curvature;
}

} // namespace planner
} // namespace adore
