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
#include "dynamics/traffic_participant.hpp"

namespace adore
{
namespace dynamics
{

void
update_traffic_participants( TrafficParticipantSet& participants, const TrafficParticipant& new_participant_data )
{
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
remove_old_participants( TrafficParticipantSet& participants, double max_age, double current_time )
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
