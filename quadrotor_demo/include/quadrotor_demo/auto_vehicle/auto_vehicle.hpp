/*
 * sensors.hpp
 *
 *  Created on: July 19, 2019
 *      Author: jfang
 */

#ifndef AUTOVEHICLE_HPP
#define AUTOVEHICLE_HPP

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>

#include "../graph/graph.hpp"
#include "../graph/astar.hpp"
#include "../map/square_grid.hpp"
#include "../auto_team/auto_team.hpp"
#include "../auto_team/auto_team_impl.hpp"
#include "tasks.hpp"

namespace librav{
    const double ENTROPY_THRED_ = 0.0243;
    const double RW_BENEFIT_ = 1e5;
    const double EPS_ = 1e-2;
    const double MEASUREMENT_COST_ = 5;
    const double NOT_CAPABILITY_COST_ = RW_BENEFIT_ - MEASUREMENT_COST_;
    const double PENALTY_ = 200.0;
    const int64_t MAX_TASKS = 1e10;

    typedef struct
    {
        std::vector<Eigen::Matrix<double,1,Eigen::Dynamic>> y_history_;
        std::vector<Eigen::Matrix<int,1,Eigen::Dynamic>> z_history_;
        std::vector<Eigen::Matrix<int,1,Eigen::Dynamic>> iteration_neighb_history_;
    }History;

    class AutoVehicle
    {
        public: 
            AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk);
            // IPAS
            AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk,int64_t num_ss);
            ~AutoVehicle(){};

            int64_t idx_;
            int64_t pos_;
            TaskType vehicle_type_;
            Eigen::Matrix<int,1,Eigen::Dynamic> network_topo_;
            double energy_capacity_;

            int64_t num_vehicles_;
            int64_t num_tasks_;

            /* IPAS */
            int64_t num_sensors_;
            std::shared_ptr<SquareGrid> local_grid_;
            std::shared_ptr<Graph_t<SquareCell*>> local_graph_;
            int64_t ipas_iter_;
            std::vector<std::pair<int64_t,double>> hotspots_; 
            // std::map<int64_t,double> nz_ig_zone_;

            std::vector<std::vector<std::pair<int64_t,double>>> history_hspots_;
            

            /* CBBA */
            Eigen::Matrix<int,1,Eigen::Dynamic> opt_pos_;
        
            std::map<int64_t, double> reward_;

            std::vector<int64_t> task_bundle_;
            std::vector<int64_t> task_path_;

            std::vector<int64_t> h_avai_;

            Eigen::Matrix<double,1,Eigen::Dynamic> cbba_y_;
            Eigen::Matrix<int,1,Eigen::Dynamic> cbba_z_;

            Eigen::Matrix<int,1,Eigen::Dynamic> iteration_neighb_;
            History cbba_history_;

            int64_t cbba_iter_;

            /* CBBA Calculation */
            void InitCBBA(int64_t num_tk);
            std::vector<int64_t> GetNeighborsIDs();

            // Bundle Construction Phase
            int64_t FindOptIndeTask();
            void AvailableTasks();
            void UpdateReward(TasksSet tasks);
            double PathCostComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> tk_path, int64_t init_pos);
            void BundleAdd(TasksSet tasks);
            
            // Bundle Remove phase
            void BundleRemove();
            void PathRemove();


            /* IPAS */
            void InitLocalGraph(std::shared_ptr<SquareGrid> grid);
            void UpdateLocalMap(std::shared_ptr<Graph_t<SquareCell*>> graph,TasksSet tasks);
            void ComputeLocalHotspots(TasksSet tasks);
            Path_t<SquareCell*> PathComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> tk_path, int64_t init_pos);

            std::vector<int64_t> SubRegionComputation(std::map<int64_t, int> config, int64_t sensor_pos, int64_t start_id, int64_t end_id);
            std::vector<int64_t> SubRegionFromPaths(std::vector<Path_t<SquareCell*>> paths);
            std::vector<int64_t> ComputeNZIGRegion(std::vector<int64_t> sub_domain);
    };

    namespace IPASMeasurement{
        // Generate the AutoTeam
        std::shared_ptr<AutoTeam_t<AutoVehicle>> ConstructAutoTeam(std::vector<AutoVehicle>& teams);
        // Generate current paths for the AutoTeam
        std::map<int64_t,Path_t<SquareCell*>> GeneratePaths(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,TasksSet tasks,TaskType task_type);
        // Check the convergence of the IPAS
        bool IPASConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> team,std::map<int64_t,Path_t<SquareCell*>> paths_map);
        // Initialize the local grid and local graph for Auto Team
        void InitLocalGraph(std::shared_ptr<AutoTeam_t<AutoVehicle>> team, std::shared_ptr<SquareGrid> grid);
        std::vector<std::map<int64_t, int>> PotentialConfiguration(std::vector<Vertex_t<SquareCell*>*> N_v_);
        // Update the local hotspots for AutoTeam
        void ComputeHotSpots(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team, TasksSet tasks);
        // Check whether the whole Auto Team is informed with the latest information
        bool MapMergeConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams);
        // Construct the measurement tasks
        TasksSet ConstructMeasurementTasks(std::shared_ptr<AutoTeam_t<AutoVehicle>> team);
        // Update the local graph for AutoTeam based on latest measurements
        void UpdateLocalMap(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,std::shared_ptr<Graph_t<SquareCell*>> true_graph,TasksSet tasks);
        void MergeLocalMap(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams);
    }


}
#endif /* AUTOVEHICLE_HPP */