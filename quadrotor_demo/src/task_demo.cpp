#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
#include <iostream>
#include <fstream>

// self-defined library
#include "../include/quadrotor_demo/graph/graph.hpp"
#include "../include/quadrotor_demo/graph/astar.hpp"
#include "../include/quadrotor_demo/map/square_grid.hpp"

#include "../include/quadrotor_demo/auto_vehicle/auto_vehicle.hpp"
#include "../include/quadrotor_demo/auto_vehicle/tasks.hpp"
#include "../include/quadrotor_demo/auto_team/auto_team.hpp"
#include "../include/quadrotor_demo/task_assignment/cbba_impl.hpp"

// ROS library

using namespace librav;


int main() {
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
                                    Task(3,5,{190},TaskType::RESCUE,num_vehicle),
                                    Task(4,6,{130},TaskType::RESCUE,num_vehicle),
                                    Task(5,7,{150},TaskType::RESCUE,num_vehicle)};
    TasksSet tasks_ = TasksSet(tasks_data_);
    
    // Auto Vehicle Team
    // Index of drone, Initial position, # of drones, Communicate network, Task Type, # of tasks
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,num_vehicle);
    int64_t num_sensors = 3;
    std::vector<AutoVehicle> drones = { AutoVehicle(0,0,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                        AutoVehicle(1,210,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                        AutoVehicle(2,224,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                        AutoVehicle(3,0,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                        AutoVehicle(4,210,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                        AutoVehicle(5,224,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors) };
    
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_ = IPASMeasurement::ConstructAutoTeam(drones);
    //===============================================================================================//
    //============================================= MAP =============================================//
    //===============================================================================================//
    // Initialize uncertain map
    int64_t num_row = 15;
    int64_t num_col = 15;
    std::shared_ptr<SquareGrid> uncertain_grid = GridGraph::CreateSquareGrid(num_row,num_col,1,vehicle_team_,tasks_);
	std::shared_ptr<Graph_t<SquareCell*>> uncertain_graph = GridGraph::BuildGraphFromSquareGrid(uncertain_grid, true);

    // Build true map
    std::shared_ptr<SquareGrid> true_grid = GridGraph::CreateSquareGrid(num_row,num_col,1);
    std::vector<std::vector<int64_t>> range_idx_ = {{30,36}};
    // {{15,20},{36,40},{56,60},{78,80},{40,48},{60,67},{80,86},
    //                                             {100,105},{120,124},{140,143},{151,160},{171,180},{191,200},
    //                                             {211,214},{217,220},{231,234},{237,240},{220,226},{240,246},
    //                                             {260,266},{327,331},{346,352},{366,372},{387,391}};
    for(auto rg: range_idx_){
        for(int ii = rg[0]; ii<rg[1];ii++){
            true_grid->SetObstacleRegionLabel(ii,1);
        }
    }
    // true_grid->SetObstacleRegionLabel(32,1);
    // true_grid->SetObstacleRegionLabel(5,1);
    // true_grid->SetObstacleRegionLabel(6,1);
    // true_grid->SetObstacleRegionLabel(9,1);
    // true_grid->SetObstacleRegionLabel(14,1);
    // true_grid->SetObstacleRegionLabel(20,1);
    // true_grid->SetObstacleRegionLabel(21,1);
    // true_grid->SetObstacleRegionLabel(22,1);
    std::shared_ptr<Graph_t<SquareCell *>> true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);
    //===============================================================================================//
    //============================================= CBBA ============================================//
    //===============================================================================================//   
    std::ofstream file_path;
    file_path.open("/home/han/quadrotordemo_ws/src/path.txt",std::ios::trunc);

    // Initialize local_grid and local_graph
    IPASMeasurement::InitLocalGraph(vehicle_team_,uncertain_grid);   
    int64_t ipas_tt = 0;
    while (true){
        ipas_tt ++;
        std::cout << "Iteration: " << ipas_tt << std::endl;
        file_path << "Iteration: " << ipas_tt << "\n";
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
            file_path << "The path for vehicle " << p.first << " is: ";
            for(auto v: p.second){
                std::cout << v->id_ <<", ";
                file_path << v->id_ <<", ";
            }
            std::cout << std::endl;
            file_path << "\n";
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
        std::cout << path_sensing_.size() << std::endl;
        for(auto p: path_sensing_){
            std::cout << "The path for sensor " << p.first << " is: ";
            file_path << "The path for sensor " << p.first << " is: ";
            for(auto v: p.second){
                std::cout << v->id_ <<", ";
                file_path << v->id_ <<", ";
            }
            std::cout << std::endl;
            file_path << "\n";
        }

        // Update initial position for sensors
        for(auto agent: vehicle_team_->auto_team_){
            if (agent->vehicle_type_ == TaskType::MEASURE && !agent->task_path_.empty()){
                Task tsk = sensing_tasks_.GetTaskFromID(agent->task_path_.back());
                agent->pos_ = tsk.pos_.front();
            }
        }
        IPASMeasurement::UpdateLocalMap(vehicle_team_,true_graph,sensing_tasks_);
        IPASMeasurement::MergeLocalMap(vehicle_team_);
        file_path.close();
    }

	return 0;
}