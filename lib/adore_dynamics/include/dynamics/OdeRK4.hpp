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

#include "dynamics/physical_vehicle_parameters.hpp"
#include <eigen3/Eigen/Dense>

namespace adore
{
namespace dynamics
{


std::vector<double>          sequence( double time_of_last_update, double integration_size, double time_current );
Eigen::Matrix<double, 10, 1> derive_10d_state_vector( double time, PhysicalVehicleParameters model,
                                                      Eigen::Matrix<double, 10, 1> vector_state_to_derive );

Eigen::Matrix<double, 10, 1> solve_for_next_state_with_ode_rk4( PhysicalVehicleParameters model, std::vector<double> time_steps_array,
                                                                Eigen::Matrix<double, 10, 1> ten_d_state );
} // namespace dynamics
} // namespace adore
