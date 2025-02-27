/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Sanath Himasekhar Konthala
 *    Giovanni Lucente
 ********************************************************************************/
#pragma once
#include "adore_map/route.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_math/point.h"
#include "adore_math/polygon.h"
#include "adore_math/shape.h"

#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace dynamics
{

enum TrafficParticipantClassification
{
  UNCLASSIFIED,
  UNKNOWN_SMALL,
  UNKNOWN_BIG,
  UNDER_DRIVABLE,
  OVER_DRIVABLE,
  PEDESTRIAN,
  BIKE,
  BICYCLE,
  MOPED,
  MOTORCYCLE,
  CAR,
  VAN,
  BUS,
  TRUCK,
  LIGHT_TRUCK,
  HEAVY_TRUCK,
  TRAM,
  TRAILER,
  SPECIAL_VEHICLE,
  PEDESTRIAN_GROUP,
  TRAIN,
  HORSE_RIDER,
  ANIMAL_SMALL,
  ANIMAL_BIG,
};

struct TrafficParticipant
{
  TrafficParticipant() {}

  // Constructor with state, id, length, width, height
  TrafficParticipant( const VehicleStateDynamic& init_state, int id, TrafficParticipantClassification classification, double length,
                      double width, double height ) :
    state( init_state ),
    bounding_box( length, width, height ),
    classification( classification ),
    id( id )
  {}

  VehicleStateDynamic              state;          // Current state of the traffic participant
  math::Box3d                      bounding_box;   // Bounding box of the traffic participant
  TrafficParticipantClassification classification; // Classification label
  int                              id;

  std::optional<math::Point2d> goal_point = std::nullopt; // Goal point
  std::optional<int>           v2x_id     = std::nullopt; // V2X ID
  std::optional<Trajectory>    trajectory = std::nullopt; // Predicted or planned trajectory
  std::optional<map::Route>    route      = std::nullopt; // Route information
};

struct TrafficParticipantSet
{
  std::unordered_map<int, TrafficParticipant> participants;
  std::optional<math::Polygon2d>              validity_area;
  void                                        update_traffic_participants( const TrafficParticipant& new_participant_data );
  void                                        remove_old_participants( double max_age, double current_time );
};


} // namespace dynamics
} // namespace adore
