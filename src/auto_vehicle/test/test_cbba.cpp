/*
 * test_reward.cpp
 *
 *  Created on: July 25, 2019
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

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"

#include "auto_vehicle/auto_vehicle.hpp"
#include "../../auto_team/auto_team.hpp"
#include "../../task_assignment/cbba_impl.hpp"

using namespace librav;

int main(int argc, char** argv )
{   
    //===============================================================================================//
    //===========================================Task - Agent========================================//
    //===============================================================================================//    
    int64_t num_vehicle = 4;
    int64_t num_tasks = 6;
    // Missions
    // Define the task information:
    // Index of task, AP value(ltl), Position, Task Type, Number of vehicle
    std::vector<Task> tasks_data_ = {Task(0,2,{75},TaskType::RESCUE,num_vehicle),
                                    Task(1,3,{84},TaskType::RESCUE,num_vehicle),
                                    Task(2,4,{135},TaskType::RESCUE,num_vehicle),
                                    Task(3,5,{156},TaskType::RESCUE,num_vehicle),
                                    Task(4,6,{109,179},TaskType::RESCUE,num_vehicle),
                                    Task(5,7,{6},TaskType::RESCUE,num_vehicle)};
    TasksSet tasks_ = TasksSet(tasks_data_);
    
    // Auto Vehicle Team
    // Index of drone, Initial position, # of drones, Communicate network, Task Type, # of tasks
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,num_vehicle);
    std::vector<AutoVehicle> drones = {AutoVehicle(0,0,num_vehicle,comm,TaskType::RESCUE,num_tasks),
                                    AutoVehicle(1,14,num_vehicle,comm,TaskType::RESCUE,num_tasks),
                                    AutoVehicle(2,210,num_vehicle,comm,TaskType::RESCUE,num_tasks),
                                    AutoVehicle(3,224,num_vehicle,comm,TaskType::RESCUE,num_tasks)};
    
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_ = IPASMeasurement::ConstructAutoTeam(drones);
    //===============================================================================================//
    //============================================= MAP =============================================//
    //===============================================================================================//
    int64_t num_row = 15;
    int64_t num_col = 15;
    int64_t cell_size = 1;
    std::shared_ptr<SquareGrid> true_grid = GridGraph::CreateSquareGrid(num_row,num_col,cell_size);
    std::shared_ptr<Graph_t<SquareCell *>> true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);
    //===============================================================================================//
    //============================================= CBBA ============================================//
    //===============================================================================================//      
    CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_,true_graph);
	return 0;
}