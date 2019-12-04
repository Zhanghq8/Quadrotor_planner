#ifndef AUTO_TEAM_IMPL_HPP
#define AUTO_TEAM_IMPL_HPP

#include <cstdint>
#include <algorithm>
#include "auto_team.hpp"

namespace librav{
    template<typename AgentType>
    AgentType* AutoTeam_t<AgentType>::GetVehicleFromID(int64_t id){
        for(auto& ag: auto_team_){
            if(ag->idx_ == id){
                return ag;
            }
        }
    }
}

#endif /* AUTO_TEAM_IMPL_HPP */