/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Sanath Himasekhar Konthala
 *    Giovanni Lucente
 *    Marko Mizdrak
 ********************************************************************************/
#include "dynamics/traffic_participant.hpp"

namespace adore
{
namespace dynamics
{

void
TrafficParticipantSet::update_traffic_participants( const TrafficParticipant& new_participant_data )
{
  // check if participant is within the validity area
  if( validity_area && !validity_area->point_inside( new_participant_data.state ) )
    return;

  if( participants.count( new_participant_data.id ) == 0 )
  {
    participants[new_participant_data.id] = new_participant_data;
    return;
  }

  participants[new_participant_data.id].state = new_participant_data.state;

  if( new_participant_data.goal_point.has_value() )
  {
    participants[new_participant_data.id].goal_point = new_participant_data.goal_point.value();
  }
  if( new_participant_data.trajectory.has_value() )
  {
    participants[new_participant_data.id].trajectory = new_participant_data.trajectory.value();
  }
  if( new_participant_data.route.has_value() )
  {
    participants[new_participant_data.id].route = new_participant_data.route.value();
  }
};

void
TrafficParticipantSet::remove_old_participants( double max_age, double current_time )
{
  for( auto it = participants.begin(); it != participants.end(); )
  {
    if( current_time - it->second.state.time > max_age )
    {
      it = participants.erase( it );
    }
    else
    {
      ++it;
    }
  }
};

} // namespace dynamics
} // namespace adore
