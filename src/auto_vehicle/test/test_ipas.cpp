/*
 * test_ipas.cpp
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
    int64_t num_vehicle = 8;
    int64_t num_tasks = 10;
    // Tasks 
    // Define the task information:
    // Index of task, AP value(ltl), Position, Task Type, Number of vehicle
    std::vector<Task> tasks_data_ = {Task(0,2,{880},TaskType::RESCUE,num_vehicle),
                                    Task(1,3,{457},TaskType::RESCUE,num_vehicle),
                                    Task(2,4,{194},TaskType::RESCUE,num_vehicle),
                                    Task(3,5,{108},TaskType::RESCUE,num_vehicle),
                                    Task(4,6,{145},TaskType::RESCUE,num_vehicle),
                                    Task(5,7,{356},TaskType::RESCUE,num_vehicle),
                                    Task(6,8,{290},TaskType::RESCUE,num_vehicle),
                                    Task(7,9,{505},TaskType::RESCUE,num_vehicle),
                                    Task(8,10,{565},TaskType::RESCUE,num_vehicle),
                                    Task(9,11,{865},TaskType::RESCUE,num_vehicle)};
    TasksSet tasks_ = TasksSet(tasks_data_);
    
    // Auto Vehicle Team
    // Index of drone, Initial position, # of drones, Communicate network, Task Type, # of tasks
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,num_vehicle);
    int64_t num_sensors = 4;
    std::vector<AutoVehicle> drones = {AutoVehicle(0,0,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    AutoVehicle(1,29,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    AutoVehicle(2,870,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    AutoVehicle(3,899,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    AutoVehicle(4,0,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    AutoVehicle(5,29,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    AutoVehicle(6,870,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    AutoVehicle(7,899,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors)};
    
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_ = IPASMeasurement::ConstructAutoTeam(drones);
    //===============================================================================================//
    //============================================= MAP =============================================//
    //===============================================================================================//
    // Initialize uncertain map
    int64_t num_row = 30;
    int64_t num_col = 30;
    std::shared_ptr<SquareGrid> uncertain_grid = GridGraph::CreateSquareGrid(num_row,num_col,1,vehicle_team_,tasks_);
	std::shared_ptr<Graph_t<SquareCell*>> uncertain_graph = GridGraph::BuildGraphFromSquareGrid(uncertain_grid, true);

    // Build true map
    std::shared_ptr<SquareGrid> true_grid = GridGraph::CreateSquareGrid(num_row,num_col,1);
    std::vector<std::vector<int64_t>> range_idx_ = {{4,6},{26,29},{34,38},{40,42},{56,59},{64,68},{86,89},
                                                {95,98},{116,119},{126,128},{156,158},{167,169},{187,189},
                                                {196,199},{217,219},{222,227},{240,243},{248,250},{253,256},
                                                {270,273},{313,316},{344,348},{352,355},{363,367},{372,379},
                                                {384,390},{393,397},{402,409},{414,420},{433,442},{459,473},
                                                {483,485},{491,503},{511,516},{526,534},{542,547},{556,564},
                                                {568,570},{573,575},{589,594},{596,600},{606,608},{615,617},
                                                {621,624},{626,630},{656,660},{687,690},{697,699},{701,703},
                                                {718,720},{731,733},{761,766},{793,795},{823,825},{890,893}};
    for(auto rg: range_idx_){
        for(int ii = rg[0]; ii<rg[1];ii++){
            true_grid->SetObstacleRegionLabel(ii,1);
        }
    }
    true_grid->SetObstacleRegionLabel(11,1);
    true_grid->SetObstacleRegionLabel(70,1);
    true_grid->SetObstacleRegionLabel(147,1);
    true_grid->SetObstacleRegionLabel(177,1);
    true_grid->SetObstacleRegionLabel(322,1);
    true_grid->SetObstacleRegionLabel(423,1);
    true_grid->SetObstacleRegionLabel(450,1);
    true_grid->SetObstacleRegionLabel(453,1);
    true_grid->SetObstacleRegionLabel(481,1);
    true_grid->SetObstacleRegionLabel(576,1);
    true_grid->SetObstacleRegionLabel(586,1);
    true_grid->SetObstacleRegionLabel(604,1);
    true_grid->SetObstacleRegionLabel(637,1);
    true_grid->SetObstacleRegionLabel(667,1);
    true_grid->SetObstacleRegionLabel(728,1);
    true_grid->SetObstacleRegionLabel(758,1);
    std::shared_ptr<Graph_t<SquareCell *>> true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);
    //===============================================================================================//
    //============================================= CBBA ============================================//
    //===============================================================================================//   
    // Initialize local_grid and local_graph
    IPASMeasurement::InitLocalGraph(vehicle_team_,uncertain_grid);   
    int64_t ipas_tt = 0;
    while (true){
        ipas_tt ++;
        std::cout << "Iteration: " << ipas_tt << std::endl;
        // Implement the CBBA to determine the task assignment
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
        // Compute the path for the auto team while satisfying its local assignment
        std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
        
        //=============================================================//
        //============================== DEBUG ========================//
        //=============================================================//
        std::cout << "Convergence for task assignment is achieved." << std::endl;
        for(auto p: path_ltl_){
            std::cout << "The path for vehicle " << p.first << " is: ";
            for(auto v: p.second){
                std::cout << v->id_ <<", ";
            }
            std::cout << std::endl;
        }
        //=============================================================//
        //=============================================================//
        //=============================================================//
        
        // Check whether the IPAS convergence is achieved
        bool flag_IPAS = IPASMeasurement::IPASConvergence(vehicle_team_,path_ltl_);
        if (flag_IPAS == true) {std::cout << "The required iteration is " << ipas_tt <<std::endl; break;}
        //===============================================================================================//
        //============================================= IPAS ============================================//
        //===============================================================================================// 
        IPASMeasurement::ComputeHotSpots(vehicle_team_,tasks_);
        TasksSet sensing_tasks_ = IPASMeasurement::ConstructMeasurementTasks(vehicle_team_);
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);

        std::map<int64_t,Path_t<SquareCell*>> path_sensing_ = IPASMeasurement::GeneratePaths(vehicle_team_,sensing_tasks_,TaskType::MEASURE);
        
        // Update initial position for sensors
        for(auto agent: vehicle_team_->auto_team_){
            if (agent->vehicle_type_ == TaskType::MEASURE && !agent->task_path_.empty()){
                Task tsk = sensing_tasks_.GetTaskFromID(agent->task_path_.back());
                agent->pos_ = tsk.pos_.front();
            }
        }

        IPASMeasurement::UpdateLocalMap(vehicle_team_,true_graph,sensing_tasks_);
        IPASMeasurement::MergeLocalMap(vehicle_team_);
    }
	return 0;
}