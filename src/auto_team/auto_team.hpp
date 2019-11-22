#ifndef AUTO_TEAM_HPP
#define AUTO_TEAM_HPP

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

namespace librav{   
    template <typename AgentType>
    class AutoTeam_t
    {
        public:
            template <class T = AgentType>
            AutoTeam_t(){};
            ~AutoTeam_t(){};

            // General attributes
            std::vector<AgentType *> auto_team_;
            int64_t num_vehicles_;
            int64_t num_tasks_;

            std::vector<AgentType *> rescue_team_;
            std::vector<AgentType *> measure_team_;

            // Return the vehicle with same id
            AgentType* GetVehicleFromID(int64_t id);
    };
}


#endif /* AUTO_TEAM_HPP */