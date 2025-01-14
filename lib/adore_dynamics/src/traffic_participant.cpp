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
update_traffic_participants( TrafficParticipantSet& participants, const TrafficParticipant& new_participant_data)
{
  auto participant_found = participants.find(new_participant_data.id);
  if (participant_found != participants.end())
  {
    participants[new_participant_data.id].state = new_participant_data.state;
    if (new_participant_data.goal_point.has_value())
    {
      participants[new_participant_data.id].goal_point = new_participant_data.goal_point;
    }
  }else
  {
    participants[new_participant_data.id] = new_participant_data;
  }
};

void 
remove_old_participants( TrafficParticipantSet& participants, double max_age, double current_time)
{
  for (auto it = participants.begin(); it != participants.end(); ) 
  {
    if (current_time - it->second.state.time > max_age) 
    {
        it = participants.erase(it);  
    } else 
    {
        ++it;  
    }
  }
}; 

} // namespace dynamics
} // namespace adore
