// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <string>

// user
#include "square_grid.hpp"
#include "../graph/astar.hpp"

using namespace librav;

int main(int argc, char** argv )
{
	int64_t num_row = 4;
    int64_t num_col = 4;
    int cell_size = 1; // unit: meter
	std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col,cell_size);

	/*** 2. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, true);

    //======================================== TEST ===============================================//
    std::vector<Vertex_t<SquareCell*>*> vts = graph->GetAllVertex();
    for(auto vt: vts){
        std::cout << "Vertex " << vt->state_->id_ << ": "<< std::endl;
        std::cout << "The (x, y) : (" << vt->state_->position_.x << ", " << vt->state_->position_.y << ")" <<std::endl;
        std::cout << "The (row,col) : (" << vt->state_->coordinate_.x << ", "<< vt->state_->coordinate_.y << ")" <<std::endl; 
        std::cout << "The probability p is " << vt->state_->p_ << ", and IG is " << vt->state_->ig_ <<std::endl;
        std::cout << "The neighbors are: " <<std::endl;
        std::vector<Vertex_t<SquareCell*>*> neighbs = vt->GetNeighbours();
        for(auto nb: neighbs){
            auto ecost = vt->GetEdgeCost(nb);
            std::cout << "Vertex " << nb->state_->id_ << ". The edge cost is: " << ecost <<std::endl;
        }
        std::cout << "=============================================" <<std::endl;
    }   

    // position: center of the vertex in the actual map
    // coordinate: row and col index of the vertex in the grid map
	return 0;
}