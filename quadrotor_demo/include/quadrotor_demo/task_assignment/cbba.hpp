#ifndef CBBA_HPP
#define CBBA_HPP


// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "../graph/graph.hpp"
#include "../graph/astar.hpp"
#include "../map/square_grid.hpp"
#include "../auto_vehicle/auto_vehicle.hpp"
#include "../auto_team/auto_team.hpp"
#include "../auto_team/auto_team_impl.hpp"


namespace librav{
    namespace CBBA
    {   
        template<typename VehicleType>
        void BundleConstruction(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team,TasksSet tasks);

        template<typename VehicleType>
        void Consensus(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team);
        
        template<typename VehicleType>
        bool CheckConvergence(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team);

        template<typename VehicleType>
        std::vector<std::vector<int>> ConsensusBasedBundleAlgorithm(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team, TasksSet tasks);
    };
}


#endif /* CBBA_HPP */