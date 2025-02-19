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
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once

#include <cmath>

#include <chrono>
#include <iostream>

#include "adore_math/angles.h"
#include "adore_math/distance.h"

#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/trajectory.hpp"

namespace adore
{
namespace controllers
{

// iLQR controller class
class iLQR
{
private:

  int    horizon_steps         = 30;  // Number of time steps
  double dt                    = 0.1; // Time step duration
  double heading_weight        = 10.0;
  double lateral_weight        = 50.0;
  double longitudinal_weight   = 1.0;
  double vel_weight            = 10.0; // Velocity tracking weight
  double acc_weight            = 0.01; // Acceleration penalty weight
  double steer_weight          = 0.1;  // Steering penalty weight
  double jerk_weight           = 0.1;
  double steer_rate_weight     = 0.1;
  double convergence_threshold = 1e-6;
  int    max_iterations        = 10000;

  // Initialize variables for the backward pass
  std::vector<Eigen::MatrixXd> A_list;
  std::vector<Eigen::MatrixXd> B_list;

  // Cost derivatives
  std::vector<Eigen::VectorXd> l_x_list;
  std::vector<Eigen::VectorXd> l_u_list;
  std::vector<Eigen::MatrixXd> l_xx_list;
  std::vector<Eigen::MatrixXd> l_uu_list;
  std::vector<Eigen::MatrixXd> l_xu_list;

  // Control gains
  std::vector<Eigen::MatrixXd> K_list;
  std::vector<Eigen::VectorXd> k_list;

  static constexpr int n_u = 2;
  static constexpr int n_x = 4;

  void initialize_matrices( int T );

  double calculate_cost( const dynamics::VehicleStateDynamic& x_ref, const dynamics::VehicleStateDynamic& xt,
                         const dynamics::VehicleCommand& u, const dynamics::VehicleCommand& u_prev );

  bool line_search( double& line_step, const double min_step, const int T, const adore::dynamics::VehicleStateDynamic& current_state,
                    adore::dynamics::Trajectory& x_traj, std::vector<adore::dynamics::VehicleCommand>& u_traj,
                    const adore::dynamics::Trajectory& ref_trajectory, double last_total_cost );

  void backward_pass( const int T );

  void compute_cost_derivatives( const int T, adore::dynamics::Trajectory& x_traj, const adore::dynamics::Trajectory& ref_trajectory,
                                 std::vector<adore::dynamics::VehicleCommand>& u_traj, const dynamics::VehicleCommand& prev_u );

  void extract_dynamics_linearization( const int T, adore::dynamics::Trajectory& x_traj,
                                       std::vector<adore::dynamics::VehicleCommand>& u_traj );


  bool debug_active = false;

public:

  dynamics::PhysicalVehicleModel model;

  dynamics::Trajectory get_last_trajectory();

  dynamics::VehicleCommandLimits limits;

  // store for warm start
  std::vector<dynamics::VehicleCommand> previous_u_traj;
  dynamics::Trajectory                  previous_traj;
  void                                  warm_start( std::vector<adore::dynamics::VehicleCommand>& u_traj );

  iLQR() {};

  void set_parameters( const std::map<std::string, double>& params );

  dynamics::VehicleCommand get_next_vehicle_command( const dynamics::Trajectory&          trajectory,
                                                     const dynamics::VehicleStateDynamic& current_state );
};

} // namespace controllers
} // namespace adore
