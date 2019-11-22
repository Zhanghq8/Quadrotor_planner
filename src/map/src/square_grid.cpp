#include "map/square_grid.hpp"
#include "auto_vehicle/auto_vehicle.hpp"
using namespace librav;

std::string SquareCell::GetCellLabels(){
    return cell_labels_.GetCellLabels();
}

int32_t SquareCell::GetCellBitMap(){
    return cell_labels_.GetCellBitMap();
}


SquareGrid::SquareGrid(int32_t row_num, int32_t col_num, double cell_size, int32_t pixel_per_meter, int32_t default_label):
                num_row_(row_num),
                num_col_(col_num),
                cell_size_(cell_size),
                pixel_per_meter_(pixel_per_meter){
    assert((row_num > 0 && col_num > 0));
    grid_cells_.resize(col_num);
    for (auto &grid_col : grid_cells_)
        grid_col.resize(row_num);
    for (int32_t y = 0; y < row_num; y++){
        for (int32_t x = 0; x < col_num; x++){
            int64_t new_id = y*col_num + x;
            SquareCell *new_cell = new SquareCell(new_id, x, y, OccupancyType::FREE);
            grid_cells_[x][y] = new_cell;
            grid_cells_[x][y]->UpdateCellInfo(num_row_, num_col_, cell_size_, pixel_per_meter_);
           
            grid_cells_[x][y]->cell_labels_.SetDefaultRegionLabel(default_label);
        }
    }
    obstacle_label_ = 1;
}

SquareGrid::~SquareGrid(){
    // Remove all Square cells
    for (auto &grid_col: grid_cells_){
        for (auto &cell: grid_col)
            delete cell;
    }
}

Position2i SquareGrid::GetCoordinateFromID(int64_t id){
    int32_t y = id/num_col_;
    int32_t x = id%num_col_;
    return Position2i(x,y);
}

void SquareGrid::SetCellOccupancy(int32_t x_col, int32_t y_row, OccupancyType occ){
    grid_cells_[x_col][y_row]->occupancy_ = occ;
    if(occ == OccupancyType::FREE){
        grid_cells_[x_col][y_row]->p_ = 0.0;
    }
    else if (occ == OccupancyType::OCCUPIED){
        grid_cells_[x_col][y_row]->p_ = 1.0;
    }
    else if (occ == OccupancyType::INTERESTED){
        grid_cells_[x_col][y_row]->p_ = 0.0;
    }
}

void SquareGrid::SetCellOccupancy(int64_t id, OccupancyType occ){
    Position2i coord = GetCoordinateFromID(id);
    SetCellOccupancy(coord.x, coord.y, occ);
}

int64_t SquareGrid::GetIDFromCoordinate(int32_t x_col, int32_t y_row)
{
    return y_row * num_col_ + x_col;
}

SquareCell* SquareGrid::GetCellFromID(int64_t id){
    Position2i coord = GetCoordinateFromID(id);
    return grid_cells_[coord.x][coord.y];
}

std::vector<SquareCell *> SquareGrid::GetNeighbors(int64_t id, bool allow_diag){
    auto cell = GetCellFromID(id);
    return GetNeighbors(cell->coordinate_.x, cell->coordinate_.y, allow_diag);
}


std::vector<SquareCell *> SquareGrid::GetNeighbors(int32_t x_col, int32_t y_row, bool allow_diag){
    std::vector<SquareCell *> neighbors;
    // Consider diagonal cells
    if (allow_diag){
        for (int32_t x = x_col - 1; x <= x_col + 1; ++x){
            for (int32_t y = y_row - 1; y <= y_row + 1; ++y){
                if (x == x_col && y == y_row)
                    continue;
                if (x >= 0 && x < num_col_ && y >= 0 && y < num_row_)
                    neighbors.push_back(grid_cells_[x][y]);
            }
        }
    }
    else{
        // Not consider diagonal cells
        Position2i pos[4];
        pos[0].x = x_col;
        pos[0].y = y_row + 1;
        pos[1].x = x_col;
        pos[1].y = y_row - 1;
        pos[2].x = x_col + 1;
        pos[2].y = y_row;
        pos[3].x = x_col - 1;
        pos[3].y = y_row;

        for (int i = 0; i < 4; i++){
            if (pos[i].x >= 0 && pos[i].x < num_col_ &&
                pos[i].y >= 0 && pos[i].y < num_row_){
                    neighbors.push_back(grid_cells_[pos[i].x][pos[i].y]);
            }
        }
    }
    return neighbors;
}

void SquareGrid::SetObstacleRegionLabel(int64_t id, int8_t label){
    this->SetCellOccupancy(id, OccupancyType::OCCUPIED);
    SquareCell* current_cell = this->GetCellFromID(id);
    current_cell->cell_labels_.RemoveRegionLabel(current_cell->cell_labels_.GetDefaultRegionLabel());
    current_cell->cell_labels_.AssignRegionLabel(label);
}

void SquareGrid::SetInterestedRegionLabel(int64_t id, int8_t label){
    this->SetCellOccupancy(id, OccupancyType::INTERESTED);
    SquareCell* current_cell = this->GetCellFromID(id);
    current_cell->cell_labels_.AssignRegionLabel(label);
}

void SquareGrid::SetCellProbability(int64_t id, double p){
    Position2i vertex_coordinate = GetCoordinateFromID(id);
    grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->p_ = p;
   
    if (grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->p_ == 0.0){
        grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->occupancy_ = OccupancyType::FREE;
    }
    else if(grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->p_ == 1.0){
        SetObstacleRegionLabel(id, obstacle_label_);
    }
    else{
        grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->occupancy_ = OccupancyType::UNKNOWN;
    }
}

std::shared_ptr<SquareGrid> GridGraph::CreateSquareGrid(int32_t row_size, int32_t col_size, double cell_size){
    return std::make_shared<SquareGrid>(row_size,col_size,cell_size);
}

std::shared_ptr<SquareGrid> GridGraph::CreateSquareGrid(int64_t row_size, int64_t col_size, double cell_size, std::shared_ptr<AutoTeam_t<AutoVehicle>> teams, TasksSet tasks){
    std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(row_size,col_size,cell_size);
    for (int i = 0; i < row_size * col_size; i++){
		grid->SetCellProbability(i,0.5);
	}
    for(auto&agent: teams->auto_team_){
        grid->SetCellOccupancy(agent->pos_, OccupancyType::FREE);
    }
    for(auto&tk: tasks.tasks_){
        for(auto& pp: tk.pos_){
            grid->SetInterestedRegionLabel(pp,tk.AP_label_);	
		    grid->SetCellOccupancy(pp,OccupancyType::INTERESTED);
        }
    }
    return grid;
}


std::shared_ptr<Graph_t<SquareCell *>> GridGraph::BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid,bool ignore_obs){
    std::shared_ptr<Graph_t<SquareCell *>> graph = std::make_shared<Graph_t<SquareCell *>>();

    // DO consider obstacle
    if (ignore_obs == false){
        for (auto &cell_col : grid->grid_cells_){
		    for (auto &cell : cell_col){
			    if(cell->occupancy_ != OccupancyType::OCCUPIED){
                    int64_t current_nodeid = cell->id_;
			        std::vector<SquareCell *> neighbour_list = grid->GetNeighbors(current_nodeid,false);
                    graph->AddVertex(cell);
                    for (auto &neighbour : neighbour_list){
                        if(neighbour->occupancy_ != OccupancyType::OCCUPIED){
                            double cost = 0;
                            cost = 1.0 * (1.0 - neighbour->p_) + PENALTY_ * neighbour->p_;
                            graph->AddEdge(cell, neighbour, cost);
                            // std::cout << "Cell occupancy is " << cell->occupancy_ << std::endl;
                            // std::cout << "Neighbor occupancy is " << neighbour->occupancy_ << std::endl;
                            // std::cout << "Edge from cell " << cell->id_ << " to cell " << neighbour->id_ << " is added. " << std::endl;
                        }
                        // else{
                        //     graph->AddVertex(neighbour);
                        // }
                    }
                    
                }
                else{
                    graph->AddVertex(cell);
                }
		    }
        }
    }
    else{
        // DO NOT consider obstacle
        for (auto &cell_col : grid->grid_cells_){
		    for (auto &cell : cell_col){
                int64_t current_nodeid = cell->id_;
			    std::vector<SquareCell *> neighbour_list = grid->GetNeighbors(current_nodeid,false);
			    for (auto &neighbour : neighbour_list){
                    double cost = 0;
				    cost = 1.0 * (1.0 - neighbour->p_) + PENALTY_ * neighbour->p_;

				    graph->AddEdge(cell, neighbour, cost);
			    }		
		    }
        }
    }
	return graph;
}

double GridGraph::CalcHeuristic(SquareCell *node1, SquareCell *node2){
    int64_t dist_row = node1->coordinate_.x - node2->coordinate_.x;
    int64_t dist_col = node1->coordinate_.y - node2->coordinate_.y;
    return std::sqrt(dist_row*dist_row + dist_col*dist_col);
}

double GridGraph::CalcHeuristicUncertain(SquareCell *node1, SquareCell *node2){
    return 1.0*(1.0 - node2->p_) + PENALTY_ * node2->p_;
}

double GridGraph::EntropyPath(Path_t<SquareCell*> path,std::shared_ptr<Graph_t<SquareCell*>> graph){
    std::vector<int64_t> ids;
    for(auto& cell: path){
        ids.push_back(cell->id_);
    }
    double entropy_ = GridGraph::EntropyCalculation(ids, graph);
    return entropy_;
}

// Compuet entropy
double GridGraph::EntropyCalculation(std::vector<int64_t> subRegion, std::shared_ptr<Graph_t<SquareCell *>> graph){
    double entropy_ = 0.0;
    for (auto& cell_: subRegion){
        Vertex_t<SquareCell *>* vertex_ = graph->GetVertexFromID(cell_);
        double p = vertex_->state_->p_;
        double q = 1.0 - p;
        if (p == 0.0 || q == 0.0){
            entropy_ += 0.0;
        }
        else{
            entropy_ += -p * log10(p) - q * log10(q);
        }
    }
    return entropy_;
}



















