/*
 * test_ipas_decentralized.cpp
 *
 *  Created on: July 29, 2019
 *      Author: jfang
 */

/*** This code is only for independent tasks ***/
// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>

// self-defined library
#include "../../graph/graph.hpp"
#include "../../graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"

#include "auto_vehicle/auto_vehicle.hpp"
#include "../../auto_team/auto_team.hpp"
#include "../../task_assignment/cbba_impl.hpp"

using namespace librav;

int main(int argc, char** argv )
{   
    //===============================================================================================//
    //===========================================Task - Agent========================================//
    //===============================================================================================//
    int64_t num_vehicle = 6;
    int64_t num_tasks = 6;
    // Tasks 
    // Define the task information:
    // Index of task, AP value(ltl), Position, Task Type, Number of vehicle
    std::vector<Task> tasks_data_ = {Task(0,2,{67},TaskType::RESCUE,num_vehicle),
                                    Task(1,3,{76},TaskType::RESCUE,num_vehicle),
                                    Task(2,4,{139},TaskType::RESCUE,num_vehicle),
                                    Task(3,5,{180},TaskType::RESCUE,num_vehicle),
                                    Task(4,6,{215},TaskType::RESCUE,num_vehicle),
                                    Task(5,7,{309},TaskType::RESCUE,num_vehicle)};
    TasksSet tasks_ = TasksSet(tasks_data_);
    
    // Auto Vehicle Team
    // Index of drone, Initial position, # of drones, Communicate network, Task Type, # of tasks
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,num_vehicle);
    std::vector<AutoVehicle> drones = {AutoVehicle(0,0,num_vehicle,comm,TaskType::RESCUE,num_tasks),
                                    AutoVehicle(1,380,num_vehicle,comm,TaskType::RESCUE,num_tasks),
                                    AutoVehicle(2,399,num_vehicle,comm,TaskType::RESCUE,num_tasks),
                                    AutoVehicle(3,0,num_vehicle,comm,TaskType::MEASURE,num_tasks),
                                    AutoVehicle(4,380,num_vehicle,comm,TaskType::MEASURE,num_tasks),
                                    AutoVehicle(5,399,num_vehicle,comm,TaskType::MEASURE,num_tasks)};
    
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_ = IPASMeasurement::ConstructAutoTeam(drones);
    //===============================================================================================//
    //============================================= MAP =============================================//
    //===============================================================================================//
    // Initialize uncertain map
    int64_t num_row = 20;
    int64_t num_col = 20;
    std::shared_ptr<SquareGrid> unccertain_grid = GridGraph::CreateSquareGrid(num_row,num_col,1,vehicle_team_,tasks_);
	std::shared_ptr<Graph_t<SquareCell*>> uncertain_graph = GridGraph::BuildGraphFromSquareGrid(unccertain_grid, true);

    // Build true map
    std::shared_ptr<SquareGrid> true_grid = GridGraph::CreateSquareGrid(num_row,num_col,1);
    std::vector<std::vector<int64_t>> range_idx_ = {{15,20},{36,40},{56,60},{78,80},{40,48},{60,67},{80,86},
                                                {100,105},{120,124},{140,143},{151,160},{171,180},{191,200},
                                                {211,214},{217,220},{231,234},{237,240},{220,226},{240,246},
                                                {260,266},{327,331},{346,352},{366,372},{387,391}};
    for(auto rg: range_idx_){
        for(int ii = rg[0]; ii<rg[1];ii++){
            true_grid->SetObstacleRegionLabel(ii,1);
        }
    }
    true_grid->SetObstacleRegionLabel(4,1);
    true_grid->SetObstacleRegionLabel(5,1);
    true_grid->SetObstacleRegionLabel(6,1);
    true_grid->SetObstacleRegionLabel(9,1);
    true_grid->SetObstacleRegionLabel(14,1);
    true_grid->SetObstacleRegionLabel(20,1);
    true_grid->SetObstacleRegionLabel(21,1);
    true_grid->SetObstacleRegionLabel(22,1);
    std::shared_ptr<Graph_t<SquareCell *>> true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);
    //===============================================================================================//
    //============================================= CBBA ============================================//
    //===============================================================================================//      

    int64_t ipas_tt = 0;
    std::map<int64_t,std::vector<double>> paths_entropy_; 
    std::vector<double> map_entropy_;
    std::vector<int64_t> nz_ig_domain_;
    std::map<int,std::vector<int64_t>> subregion_red;
    while (true){
        ipas_tt ++;
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_,uncertain_graph);
        std::map<int64_t,Path_t<SquareCell*>> path_map_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,uncertain_graph,TaskType::RESCUE);
        std::cout << "Convergence for task assignment is achieved." << std::endl;
        //============================== DEBUG ========================//
        for(auto p: path_map_){
            std::cout << "The path for vehicle " << p.first << " is: ";
            for(auto v: p.second){
                std::cout << v->id_ <<", ";
            }
            std::cout << std::endl;
        }
        break;
        //===============================================================================================//
        //============================================= IPAS ============================================//
        //===============================================================================================// 
    
    }
	return 0;
}