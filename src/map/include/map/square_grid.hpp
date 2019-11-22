#ifndef SQUARE_GRID_HPP
#define SQUARE_GRID_HPP

#include <map>
#include <cmath>
#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <assert.h>
#include <time.h>
#include <eigen3/Eigen/Core>


#include "map/grid_cell_impl.hpp"
#include "ltl/cell_label.hpp"
#include "../../auto_team/auto_team.hpp"
// #include "auto_vehicle/auto_vehicle.hpp"

namespace librav{
    /*
 * Coordinate System:
 *		
 *	^	origin ------------------> x
 *	^	|
 *		|
 * row	|
 *		|
 *		|
 *	v	|
 *  v y	v 
 *		<<		   column       >>
 */    

    /*** Square Cell in known envrionment ***/   
    class AutoVehicle;
    class TasksSet;
    class SquareCell: public GridCellBase{
        public: 
            SquareCell(int64_t id, int32_t col, int32_t row, OccupancyType _occu): GridCellBase(id, col, row, _occu){
                ig_ = 0.0; 
                p_ = 0.0;
            };
            ~SquareCell() = default;
        
            // For buchi automaton
            CellLabel cell_labels_;

            // For lifted graph
            std::vector<int64_t> lifted_vertices_id_;

            // For uncertain square cell in unknown environment
            double ig_;
            double p_;

        public:
            /*** Required by buchi automaton ***/
            std::string GetCellLabels();
            int32_t GetCellBitMap();     

            // Compute the information gain for each square cell
            // void ComputeIG(std::shared_ptr<SquareGrid> grid, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> subRegion);
    };

    class SquareGrid{
        public:
            SquareGrid(int32_t row_num, int32_t col_num, double cell_size = 0.1, int32_t pixel_per_meter = 1, int32_t default_label = 0);
            ~SquareGrid();

        friend class SquareCell;

        public: 
            std::vector<std::vector<SquareCell *>> grid_cells_;

        public: 
            int32_t num_row_;
            int32_t num_col_;
            double cell_size_;
            int32_t pixel_per_meter_;

            int32_t default_label_;
            int32_t obstacle_label_;

        public: 
            Position2i GetCoordinateFromID(int64_t id);
        
            void SetCellOccupancy(int32_t x_col, int32_t y_row, OccupancyType occ);
            void SetCellOccupancy(int64_t id, OccupancyType occ);

            int64_t GetIDFromCoordinate(int32_t x_col, int32_t y_row);

	        SquareCell* GetCellFromID(int64_t id);

	        std::vector<SquareCell*> GetNeighbors(int64_t id, bool allow_diag);
            std::vector<SquareCell*> GetNeighbors(int32_t x_col, int32_t y_row, bool allow_diag);

            /*** Set region label***/
            void SetObstacleRegionLabel(int64_t id, int8_t label);
            void SetInterestedRegionLabel(int64_t id, int8_t label);

            // /*** Uncertain Grid Map ***/
            void SetCellProbability(int64_t id, double p);

            // /*** Generate random occupancy grid map ***/
            // void SetRandomOccupanyMap(double obstacle_percentage);
            // std::vector<Agent> SetRandomAgents(int num_Agents, int num_Tasks_inde, int num_Tasks_de);
            // TasksList SetRandomTasks(int num_Tasks_inde, int num_Tasks_de);
            // bool OccupancyGridMapValidity(std::vector<Agent> agents, TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph);
            // bool OccupancyGridMapValidity(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team, TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph);
            // std::shared_ptr<SquareGrid> DuplicateSquareGrid();

            // TasksSet ConstructRandomLTLTasks(int64_t num_tasks);
            // std::shared_ptr<AutoTeam_t<AutoVehicle>> ConstructRandomAutoTeam(int64_t num_actors, int64_t num_sensors, int64_t num_tasks);
    };

    namespace GridGraph{
        // Generate grid and graph with given columns and rows
        std::shared_ptr<SquareGrid> CreateSquareGrid(int32_t row_size, int32_t col_size, double cell_size);
        std::shared_ptr<Graph_t<SquareCell *>> BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid,bool ignore_obs);
        // Generate uncertain graph
        std::shared_ptr<SquareGrid> CreateSquareGrid(int64_t row_size, int64_t col_size, double cell_size, std::shared_ptr<AutoTeam_t<AutoVehicle>> teams, TasksSet tasks);
        // Update the edge cost(Required by A*)
        double CalcHeuristic(SquareCell *node1, SquareCell *node2);    
        double CalcHeuristicUncertain(SquareCell *node1, SquareCell *node2); 
    }
}


#endif /* SQUARE_GRID_HPP */