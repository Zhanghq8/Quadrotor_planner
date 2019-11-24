#include "auto_vehicle/auto_vehicle.hpp"
using namespace librav;

// General Initialization for auto vehicle in certain map
AutoVehicle::AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk):
    idx_(idx),pos_(pos),num_vehicles_(num_v),vehicle_type_(ve_cap),num_tasks_(num_tk)
{   
    network_topo_ = Eigen::MatrixXi::Zero(1,num_vehicles_);
    for(size_t ii = 0; ii < num_vehicles_; ii++){
        network_topo_(ii) = network_topo(ii);
    }
    InitCBBA(num_tk);
}

// General initialization for auto vehicle in uncertain map
AutoVehicle::AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk,int64_t num_ss):
    idx_(idx),pos_(pos),num_vehicles_(num_v),vehicle_type_(ve_cap),num_tasks_(num_tk),num_sensors_(num_ss)
{
    network_topo_ = Eigen::MatrixXi::Ones(1,num_vehicles_);
    for(size_t ii = 0; ii < num_vehicles_; ii++){
        network_topo_(ii) = network_topo(ii);
    }
    InitCBBA(num_tasks_);
    local_grid_ = nullptr;
    local_graph_ = nullptr;
    hotspots_ = {};
    // nz_ig_zone_ = {};
    history_hspots_ = {};
}

// Initialize the parameters required for task assignment(CBBA)
void AutoVehicle::InitCBBA(int64_t num_tk){
    reward_ = {};
    num_tasks_ = num_tk;

    task_bundle_ = {};
    task_path_ = {};

    opt_pos_ = -1 * Eigen::MatrixXi::Ones(1,num_tk);
    h_avai_ = {};

    cbba_y_ = Eigen::MatrixXd::Zero(1,num_tk);
    cbba_z_ = -1 * Eigen::MatrixXi::Ones(1,num_tk);   

    iteration_neighb_ = -1 * Eigen::MatrixXi::Ones(1,num_vehicles_);

    cbba_history_.y_history_ = {cbba_y_};
    cbba_history_.z_history_ = {cbba_z_};
    cbba_history_.iteration_neighb_history_ = {iteration_neighb_};

    cbba_iter_ = 0;
}

std::vector<int64_t> AutoVehicle::GetNeighborsIDs(){
    std::vector<int64_t> neigbs = {};
    for(int ii=0; ii<num_vehicles_;ii++){
        if(ii != idx_ && network_topo_(ii)==1){
            neigbs.push_back(ii);
        }
    }
    return neigbs;
}

// Update the rewards for the available tasks
void AutoVehicle::UpdateReward(TasksSet tasks){
    // Find the task which have not in the task bundle
    std::vector<int64_t> tk_NotInBunlde = {};
    for(auto& tk_: tasks.tasks_){
        std::vector<int64_t>::iterator it_tk = std::find(task_bundle_.begin(),task_bundle_.end(),tk_.idx_);
        if(it_tk == task_bundle_.end()){
            tk_NotInBunlde.push_back(tk_.idx_);
        }
    }

    for(int64_t jj = 0; jj < tk_NotInBunlde.size(); jj++){
        // Find all possible insert positions along current task path
        std::vector<std::vector<int64_t>> dup_path_(task_path_.size()+1,task_path_);
        std::vector<double> poss_reward_ = {};
        for(int64_t mm = 0; mm < dup_path_.size(); mm++){
            std::vector<int64_t>::iterator it_insert = dup_path_[mm].begin();
            dup_path_[mm].insert(it_insert+mm,tk_NotInBunlde[jj]);

            // Compute the path cost corresponding to the duplicated task path
            double path_cost = PathCostComputation(tasks,local_graph_,dup_path_[mm],pos_);
            poss_reward_.push_back(path_cost);
        }
        double rw_min = 1e10;
        for(int64_t kk = 0; kk < poss_reward_.size(); kk++){
            if(poss_reward_[kk] < rw_min){
                rw_min = poss_reward_[kk];
                opt_pos_(tk_NotInBunlde[jj]) = kk;
            }
        }
        if (vehicle_type_ == TaskType::MEASURE){
            reward_[tk_NotInBunlde[jj]] = RW_BENEFIT_ - rw_min;
            // reward_[tk_NotInBunlde[jj]] = RW_BENEFIT_ - rw_min -  MEASUREMENT_COST_ * (opt_pos_(tk_NotInBunlde[jj]) + 1);
            // reward_[tk_NotInBunlde[jj]] = RW_BENEFIT_ - rw_min -  MEASUREMENT_COST_ * task_path_.size();
        }
        else{
            reward_[tk_NotInBunlde[jj]] = RW_BENEFIT_ - rw_min;
        }
    }
}

// Return the index of optimal task with maximum reward
int64_t AutoVehicle::FindOptIndeTask(){
    int64_t opt_tk = -1;
    // Find available tasks
    AvailableTasks();

    double max_rw = 0.0;
    if(!h_avai_.empty()){
        for(auto&tk_idx: h_avai_){
            if(reward_[tk_idx] > max_rw){
                max_rw = reward_[tk_idx];
                opt_tk = tk_idx;
            }
        }
    }
    return opt_tk;
}

// Update the available tasks which can be assigned the auto vehicle
void AutoVehicle::AvailableTasks(){
    // Initialize h_avai
    h_avai_.clear();
    std::map<int64_t,double>::iterator it_tk;
    for(it_tk = reward_.begin(); it_tk != reward_.end(); it_tk++){
        if(it_tk->second - cbba_y_(it_tk->first) > EPS_){
            h_avai_.push_back(it_tk->first);
        }
        else if(std::fabs(it_tk->second - cbba_y_(it_tk->first))<= EPS_){
            if(idx_ < cbba_z_(it_tk->first)){
                h_avai_.push_back(it_tk->first);
            }
        }
        else{
            continue;
        }
    }
}

double AutoVehicle::PathCostComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> tk_path, int64_t init_pos){
    double path_cost_ = 0.0;
    Path_t<SquareCell*> route = PathComputation(tasks,graph,tk_path,init_pos);
    if(route.empty()){
        for(auto tk_idx: tk_path){
            Task task = tasks.GetTaskFromID(tk_idx);
            if(vehicle_type_ != task.task_type_){
                return NOT_CAPABILITY_COST_;
            }
        }
        return 0.0;
    }
   
    // path_cost_ = double(path.size());
    if(vehicle_type_ == TaskType::RESCUE){
        for(int64_t kk = 0; kk < route.size()-1; kk++){
            path_cost_ += GridGraph::CalcHeuristicUncertain(route[kk],route[kk+1]);
        }
    }
    else if (vehicle_type_ == TaskType::MEASURE){
        path_cost_ = double(route.size());
    }
    return path_cost_;
    
}

void AutoVehicle::BundleRemove(){
    bool flag_outbid = 0;
    if(!task_bundle_.empty()){
        for(int64_t jj = 0; jj < task_bundle_.size(); jj++){
            if(cbba_z_(task_bundle_[jj]) != idx_){
                flag_outbid = 1;
            }
            if (flag_outbid == 1){
                if(cbba_z_(task_bundle_[jj]) == idx_){
                    cbba_z_(task_bundle_[jj]) = -1;
                    cbba_y_(task_bundle_[jj]) = -1;
                }
                task_bundle_[jj] = -1;
            }
        }
        task_bundle_.erase(std::remove(task_bundle_.begin(),task_bundle_.end(),-1),task_bundle_.end());
    }
    PathRemove();
}

// Update the task path
void AutoVehicle::PathRemove(){
    if(!task_path_.empty()){
        std::vector<int64_t> duplicated_path_ = task_path_;
        for(auto& tk: duplicated_path_){
            std::vector<int64_t>::iterator it_remove = std::find(task_bundle_.begin(),task_bundle_.end(),tk);
            if(it_remove == task_bundle_.end()){
                task_path_.erase(std::remove(task_path_.begin(),task_path_.end(),tk),task_path_.end());
            }
        }
    }
}
    
void AutoVehicle::BundleAdd(TasksSet tasks){
    while(task_bundle_.size() <= MAX_TASKS){
        // Update the reward 
        UpdateReward(tasks);
        int64_t opt_task_ = FindOptIndeTask();
        if(opt_task_ != -1){
            cbba_z_(opt_task_) = idx_;
            cbba_y_(opt_task_) = reward_[opt_task_];

            std::vector<int64_t>::iterator it_path = task_path_.begin();
            task_path_.insert(it_path+opt_pos_(opt_task_),opt_task_);
            task_bundle_.push_back(opt_task_);
        }
        else{
            break;
        }
    }
    cbba_history_.z_history_.push_back(cbba_z_);
    cbba_history_.y_history_.push_back(cbba_y_);  
}

// Compute the path while satisfying the current task assignment
Path_t<SquareCell*> AutoVehicle::PathComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> tk_path, int64_t init_pos){
    Path_t<SquareCell*> path;
    if (tk_path.empty()){
        return path;
    }
    else{
        Task final_tk = tasks.GetTaskFromID(tk_path.back());
        int64_t start_idx = init_pos;
        for(auto& tk_idx: tk_path){
            Task task = tasks.GetTaskFromID(tk_idx);
            // Whether vehicle is able to acomplish the task
            if(vehicle_type_ == task.task_type_){
                for(auto& pos: task.pos_){
                    Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(pos);
                    int64_t target_idx = vert->state_->id_;
                    Path_t<SquareCell*> path_seg;
                    if(vehicle_type_ == TaskType::RESCUE){
                        path_seg = AStar::Search(graph,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristicUncertain));
                    }
                    else if (vehicle_type_ == TaskType::MEASURE){
                        path_seg = AStar::SearchIgnoreEdgeCost(graph,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
                    }
                    
                    if(target_idx == final_tk.pos_.back()){
                        path.insert(path.end(), path_seg.begin(), path_seg.end());
                    }
                    else{
                        path.insert(path.end(), path_seg.begin(), path_seg.end()-1);
                    }
                    start_idx = target_idx;
                }
            }
        }
    }
    return path;
}

void AutoVehicle::InitLocalGraph(std::shared_ptr<SquareGrid> grid){
    local_grid_ = grid->DuplicateSquareGrid();
    local_graph_ = GridGraph::BuildGraphFromSquareGrid(local_grid_,true);
}

void AutoVehicle::ComputeLocalHotspots(TasksSet tasks){
    hotspots_ = {};
    
    if (!task_path_.empty()){
        // Determine the path segment from the route which satisfy the local assignment
        std::vector<Path_t<SquareCell*>> route_segs_;
        int64_t start_pos = pos_;
        for(auto tg: task_path_){
            route_segs_.push_back(PathComputation(tasks,local_graph_,{tg},start_pos));
            start_pos = tasks.GetTaskPosFromID(tg);
        }
        std::vector<int64_t> sub_domain = AutoVehicle::SubRegionFromPaths(route_segs_);
        std::vector<int64_t> nzig = AutoVehicle::ComputeNZIGRegion(sub_domain);

        for(size_t ii = 0; ii < local_grid_->num_col_* local_grid_->num_row_; ii++){
            std::vector<int64_t>::iterator it_ig = std::find(nzig.begin(),nzig.end(),ii);
            Vertex_t<SquareCell*>* vt_ig = local_graph_->GetVertexFromID(ii);
            if(it_ig == nzig.end()){
                vt_ig->state_->ig_ = 0.0;
            }
            else{
                vt_ig->state_->ComputeIG(local_grid_, local_graph_, sub_domain);
            }
        }
        // Sensors position selection
        std::vector<int64_t> sensor_pos = GridGraph::SelectSensorsPos(num_sensors_, local_graph_);
        for(auto& ss: sensor_pos){
            Vertex_t<SquareCell*>* vert = local_graph_->GetVertexFromID(ss);
            std::pair<int,double> hspot = std::make_pair(ss,vert->state_->ig_);
            hotspots_.push_back(hspot); 
        }
    }
    else{
        for(size_t ii = 0; ii < local_grid_->num_col_* local_grid_->num_row_; ii++){
            Vertex_t<SquareCell*>* vt_ig = local_graph_->GetVertexFromID(ii);
            vt_ig->state_->ig_ = 0.0;
        }
    }
    // Update the history of hot spots
    history_hspots_.push_back(hotspots_);

    //===========================================================//
    //========================= DEBUG ===========================//
    //===========================================================//
    std::cout << "The hotspots of vehicle " << idx_ << " is :" << std::endl;
    for(auto&spot:hotspots_){
        std::cout << spot.first << " (" << spot.second << ")" << std::endl;
    }
}


std::vector<int64_t> AutoVehicle::SubRegionFromPaths(std::vector<Path_t<SquareCell*>> paths){
    std::vector<int64_t> subRegion;
    for(auto& segment: paths){
        // Add the cells along current routes into sub-region
        std::vector<int64_t> segment_id = {};
  
        for(auto& cell: segment){
            std::vector<int64_t>::iterator it = std::find(subRegion.begin(), subRegion.end(), cell->id_);
            if(it == subRegion.end()){
                subRegion.push_back(cell->id_);
            }
            segment_id.push_back(cell->id_);
        }
        
        // Looking for cells along potential routes
        for(auto & cell: segment){
            // Find a set of vertices 
            Vertex_t<SquareCell*>* vert_ = local_graph_->GetVertexFromID(cell->id_);
            std::vector<Vertex_t<SquareCell*>*> neighbors = vert_->GetNeighbours();
            std::vector<Vertex_t<SquareCell*>*> unknown_verts;
            for(auto& neigb: neighbors){
                if(neigb->state_->occupancy_ == OccupancyType::UNKNOWN){
                    unknown_verts.push_back(neigb);
                }
            }
            if (vert_->state_->occupancy_ == OccupancyType::UNKNOWN){
                unknown_verts.push_back(vert_);
            }

            // Find the index of unknown verts along the current segment
            std::map<int64_t,Vertex_t<SquareCell*>*> un_v_segment;
            if(!unknown_verts.empty()){
                for(auto& un_v:unknown_verts){
                    std::vector<int64_t>::iterator it_segm = std::find(segment_id.begin(), segment_id.end(), un_v->state_->id_);
                    if(it_segm != segment_id.end()){
                        int64_t v_idx = it_segm - segment_id.begin();
                        un_v_segment[v_idx] = un_v;
                    }
                }
            
                // Compute the src and dst vertex along the segment
                if (!un_v_segment.empty()){
                    int64_t init_idx = un_v_segment.begin()->first;
                    int64_t start_idx = 0;
                    if(init_idx == 0){
                        start_idx = 0;
                    }
                    else{
                        start_idx = init_idx - 1;
                    }
                    int64_t start_id = segment_id[start_idx];

                    int64_t end_idx = un_v_segment.rbegin()->first;
                    int64_t finish_idx = segment_id.size() - 1;
                    if(end_idx == finish_idx){
                        continue;
                    }
                    else{
                        finish_idx = end_idx + 1;
                    }
                    int64_t finish_id = segment_id[finish_idx];
                   

                    // Compute the repaired route from src and dst
                    std::vector<std::map<int64_t, int>> binary_config = IPASMeasurement::PotentialConfiguration(unknown_verts);
                    for(auto& potential_binary: binary_config){
                        std::vector<int64_t> region_ = AutoVehicle::SubRegionComputation(potential_binary,vert_->state_->id_,start_id,finish_id);
                        for(auto& cell_: region_){
                            std::vector<int64_t>::iterator it_reg = std::find(subRegion.begin(), subRegion.end(), cell_);
                            if(it_reg == subRegion.end()){
                                subRegion.push_back(cell_);
                            }
                        }
                    }
                }
                else{
                    break;
                }
            }
        }
    }
    return subRegion;
}

std::vector<int64_t> AutoVehicle::SubRegionComputation(std::map<int64_t, int> config, int64_t sensor_pos, int64_t start_id, int64_t end_id){
     // Create a duplicate grid 
    std::shared_ptr<SquareGrid> duplicated_grid = local_grid_->DuplicateSquareGrid();

    for(auto& un_cell: config){
        double previous_bel = duplicated_grid->GetCellFromID(un_cell.first)->p_;
        // If measurement is obstacle
        if (un_cell.second == 1){
            if (un_cell.first != sensor_pos){
                double bel_obs = GridGraph::UpdateBelProbability(previous_bel,1);
                duplicated_grid->SetCellProbability(un_cell.first, bel_obs);
            }
            else {
                duplicated_grid->SetObstacleRegionLabel(un_cell.first, local_grid_->obstacle_label_);
                duplicated_grid->SetCellProbability(un_cell.first, 1.0);
            }
        }
        else{
            // Q: Should we update the edge here??
            // If the measurement is free
            if(un_cell.first != sensor_pos){
                double bel_free = GridGraph::UpdateBelProbability(previous_bel,0);
                duplicated_grid->SetCellProbability(un_cell.first, bel_free);
            }
            else{
                duplicated_grid->SetCellOccupancy(un_cell.first, OccupancyType::FREE);
                duplicated_grid->SetCellProbability(un_cell.first, 0.0);
            }
        }
    }
    // Build the graph corresponding to the duplicated grid
    std::shared_ptr<Graph_t<SquareCell *>> duplicated_graph = GridGraph::BuildGraphFromSquareGrid(duplicated_grid, false);
    Path_t<SquareCell*> path_ = AStar::Search(duplicated_graph,start_id,end_id,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
    std::vector<int64_t> region_;
    if(!path_.empty()){
        for(auto& cell:path_){
            region_.push_back(cell->id_);
        }
    }
    return region_;
}

// Update the region with none zero ig value
std::vector<int64_t> AutoVehicle::ComputeNZIGRegion(std::vector<int64_t> sub_domain){
    std::vector<int64_t> nzig = sub_domain;
    for(auto&vt: sub_domain){
        Vertex_t<SquareCell*>* vert_ = local_graph_->GetVertexFromID(vt);
        std::vector<SquareCell*> neighbors = local_grid_->GetNeighbors(vt,false);
        for(auto& negb_vt: neighbors){
            std::vector<int64_t>::iterator it_v = std::find(nzig.begin(),nzig.end(),negb_vt->id_);
            if(it_v == nzig.end()){
                nzig.push_back(negb_vt->id_);
            }
        }
    }
    return nzig;
}

void AutoVehicle::UpdateLocalMap(std::shared_ptr<Graph_t<SquareCell*>> true_graph,TasksSet tasks){
    if(!task_path_.empty()){
        for(auto&tk_id: task_path_){
            // Find the center of sensor
            Task task = tasks.GetTaskFromID(tk_id);
            Vertex_t<SquareCell*>* true_cs= true_graph->GetVertexFromID(task.pos_.front());
            Vertex_t<SquareCell*>* uncertain_cs = local_graph_->GetVertexFromID(task.pos_.front());
            uncertain_cs->state_->occupancy_ = true_cs->state_->occupancy_;
            if(uncertain_cs->state_->occupancy_ == OccupancyType::FREE || uncertain_cs->state_->occupancy_ == OccupancyType::INTERESTED){
                uncertain_cs->state_->p_ = 0.0;
            }
            else if(uncertain_cs->state_->occupancy_ == OccupancyType::OCCUPIED){
                uncertain_cs->state_->p_ = 1.0;
            }

            //Update the vertices inside of the sensing range
            std::vector<Vertex_t<SquareCell*>*> neighbors = uncertain_cs->GetNeighbours();
            for(auto& neigb: neighbors){
                Vertex_t<SquareCell*>* true_neigb = true_graph->GetVertexFromID(neigb->state_->id_);
                if(neigb->state_->occupancy_ == OccupancyType::UNKNOWN){
                    if(true_neigb->state_->occupancy_ == OccupancyType::FREE || true_neigb->state_->occupancy_ == OccupancyType::INTERESTED){
                        double belif_p = GridGraph::UpdateBelProbability(neigb->state_->p_,0);
                        neigb->state_->p_ = belif_p;
                        local_grid_->SetCellProbability(neigb->state_->id_,belif_p);
                        if(neigb->state_->p_ <= cell_accurancy_){
                            neigb->state_->occupancy_ = OccupancyType::FREE;
                            neigb->state_->p_ = 0.0;
                        }
                        if (neigb->state_->p_ > 1.0 - cell_accurancy_){
                            neigb->state_->occupancy_ = OccupancyType::OCCUPIED;
                            neigb->state_->p_ = 1.0;
                        }
                    }
                    else if (true_neigb->state_->occupancy_ == OccupancyType::OCCUPIED){
                        double belif_p = GridGraph::UpdateBelProbability(neigb->state_->p_,1);
                        neigb->state_->p_ = belif_p;
                        if(neigb->state_->p_ <= cell_accurancy_){
                            neigb->state_->occupancy_ = OccupancyType::FREE;
                            neigb->state_->p_ = 0.0;
                        }
                        if (neigb->state_->p_ > 1.0 - cell_accurancy_){
                            neigb->state_->occupancy_ = OccupancyType::OCCUPIED;
                            neigb->state_->p_ = 1.0;
                        }
                    }
                }
            }
        }

        // Update the cost of local graph
        if (vehicle_type_ == TaskType::RESCUE){
            auto edges = local_graph_->GetAllEdges();
            std::vector<int64_t> rm_dst;
            for (auto& e: edges){
                Vertex_t<SquareCell*>* dst_v = e->dst_;
                double uncertain_cost = 1.0 * (1.0 - dst_v->state_->p_) + PENALTY_*dst_v->state_->p_;
                e->cost_ = uncertain_cost;
                // remove the edge points to obstacle
                if(dst_v->state_->p_ == 1.0){
                    rm_dst.push_back(dst_v->state_->id_);
                }
            }
        }
    }
}

//================================================================================================//
//=============================================== IPAS ===========================================//
//================================================================================================//
std::shared_ptr<AutoTeam_t<AutoVehicle>> IPASMeasurement::ConstructAutoTeam(std::vector<AutoVehicle>& team){
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_group = std::make_shared<AutoTeam_t<AutoVehicle>>();
    for(auto& ve_:team){
        vehicle_group->auto_team_.push_back(&ve_);
    }
    // Organize the team
    vehicle_group->measure_team_ = {};
    vehicle_group->rescue_team_ = {};
    for(auto&agent: vehicle_group->auto_team_){
        if(agent->vehicle_type_ == TaskType::MEASURE){
            vehicle_group->measure_team_.push_back(agent);
        }
        else{
            vehicle_group->rescue_team_.push_back(agent);
        }
    }
    vehicle_group->num_vehicles_ = vehicle_group->auto_team_.size();
    vehicle_group->num_tasks_ = vehicle_group->auto_team_.front()->num_tasks_;
    return vehicle_group;
}

std::map<int64_t,Path_t<SquareCell*>> IPASMeasurement::GeneratePaths(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,TasksSet tasks,TaskType task_type){
    std::map<int64_t,Path_t<SquareCell*>> paths_map_;
    for(auto agent: vehicle_team->auto_team_){
        if (agent->vehicle_type_ == task_type){
            if(!agent->task_path_.empty()){
                paths_map_[agent->idx_] = agent->PathComputation(tasks,agent->local_graph_,agent->task_path_,agent->pos_);
            }
            else{
                Path_t<SquareCell*> empty_path_ = {};
                paths_map_[agent->idx_] = empty_path_;
            }
        }
        else{
            Path_t<SquareCell*> empty_path_ = {};
            paths_map_[agent->idx_] = empty_path_;
        }    
    }
    return paths_map_;
}

bool IPASMeasurement::IPASConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> team,std::map<int64_t,Path_t<SquareCell*>> paths_map){
    for(auto vehicle: team->auto_team_){
        if(paths_map[vehicle->idx_].empty()){return true;}
        double en_thre = paths_map[vehicle->idx_].size() * ENTROPY_THRED_;
        double entropy_path = GridGraph::EntropyPath(paths_map[vehicle->idx_],vehicle->local_graph_);
        if(entropy_path > en_thre){return false;}
    }
    return true;
}

void IPASMeasurement::InitLocalGraph(std::shared_ptr<AutoTeam_t<AutoVehicle>> team, std::shared_ptr<SquareGrid> grid){
    for(auto vehicle:team->auto_team_){
        vehicle->InitLocalGraph(grid);
    }
}

// Given a list of unknown vertices, compute the list of possible configuration
std::vector<std::map<int64_t, int>> IPASMeasurement::PotentialConfiguration(std::vector<Vertex_t<SquareCell*>*> N_v_){
    int num_confg = std::pow(2, N_v_.size());
    std::vector<std::map<int64_t,int>> binary_config_;
    for (int i = 0; i < num_confg; i++){
        int single_case_ = i;
        std::vector<int> config_ = {};
        while(single_case_ > 0){
            config_.insert(config_.begin(), single_case_%2);
            single_case_ = single_case_/2;
        }
        while (config_.size() < N_v_.size()){
            config_.insert(config_.begin(),0);
        }
        std::map<int64_t, int> single_config_ = {};
        for(int j = 0; j < N_v_.size(); j++){
            single_config_[N_v_[j]->state_->id_] = config_[j];
        }
        binary_config_.push_back(single_config_);
    } 
    return binary_config_;
}

void IPASMeasurement::ComputeHotSpots(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team, TasksSet tasks){
    for(auto&agent: vehicle_team->auto_team_){
        agent->ComputeLocalHotspots(tasks);
    }

    std::map<int64_t,double> mm;
    for(auto agent: vehicle_team->auto_team_){
        for(auto hspt: agent->hotspots_){
            if(mm.count(hspt.first)==0){mm[hspt.first] = hspt.second;}
        }
    }
    int64_t cont = 0;
    std::vector<std::pair<int64_t,double>> consensus_hspts_={};
    while(consensus_hspts_.size() < vehicle_team->auto_team_[0]->num_sensors_){
        double max_hspt = -1e3;
        int64_t max_hspt_idx = -1;
        for(auto cc: mm){
            if(cc.second - max_hspt > 1e-6){
                max_hspt = cc.second;
                max_hspt_idx = cc.first;
            }
            else if (fabs(cc.second - max_hspt) <= 1e-6){
                if(cc.first < max_hspt_idx){
                    max_hspt_idx = cc.first;
                }
            }
            else{continue;}
        }
        
        std::pair<int64_t,double> ig_pair = std::make_pair(max_hspt_idx,max_hspt);
        consensus_hspts_.push_back(ig_pair);
        mm.erase(max_hspt_idx);
    }

    for(auto &agent: vehicle_team->auto_team_){
        agent->hotspots_ = consensus_hspts_;
        agent->history_hspots_.push_back(agent->hotspots_);
    }
   
}

bool IPASMeasurement::MapMergeConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams){
    bool flag_ = true;
    for(auto& agent: teams->auto_team_){
        for(int64_t ii = 0; ii < teams->auto_team_.size(); ii++){
            if(agent->iteration_neighb_(ii) != 1){
                return false;
            }
        }
    }
    return flag_;
}

TasksSet IPASMeasurement::ConstructMeasurementTasks(std::shared_ptr<AutoTeam_t<AutoVehicle>> team){
    // Create the measurement tasks
    std::vector<Task> tasks;
    for(int64_t kk = 0; kk < team->auto_team_[0]->hotspots_.size(); kk++){
        std::pair<int64_t,double> hspot = team->auto_team_[0]->hotspots_[kk];
        std::cout << "The hspt is " << hspot.first << std::endl;
        Task task(kk,0,{hspot.first},TaskType::MEASURE,1);
        tasks.push_back(task);
    }
    TasksSet tasks_(tasks);
    return tasks_;
}

void IPASMeasurement::UpdateLocalMap(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,std::shared_ptr<Graph_t<SquareCell*>> true_graph,TasksSet tasks){
    for(auto vehicle: vehicle_team->auto_team_){
        vehicle->UpdateLocalMap(true_graph,tasks);
    }
}

void IPASMeasurement::MergeLocalMap(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams){
    bool flag_ = false;
    for(auto&agent: teams->auto_team_){
        agent->iteration_neighb_ = Eigen::MatrixXi::Zero(1,teams->auto_team_.size());
        agent->iteration_neighb_(agent->idx_) = 1;
    }
    while (flag_ != true){
        for(auto&agent: teams->auto_team_){
            std::vector<int64_t> neighbors = agent->GetNeighborsIDs();
            for(auto&nb_id: neighbors){
                AutoVehicle* neighbor = teams->GetVehicleFromID(nb_id);
                for(auto& hspot: agent->hotspots_){
                    // Find the set of vertices which are required to update
                    std::vector<Vertex_t<SquareCell*>*> vert_to_upadte = {};
                    Vertex_t<SquareCell*>* vt = agent->local_graph_->GetVertexFromID(hspot.first);
                    if(vt->state_->occupancy_ == OccupancyType::UNKNOWN){
                        vert_to_upadte.push_back(vt);
                    }
                    std::vector<Vertex_t<SquareCell*>*> vt_neigbs = vt->GetNeighbours();
                    for(auto&vt_nb:vt_neigbs){
                        if(vt_nb->state_->occupancy_ == OccupancyType::UNKNOWN){
                            vert_to_upadte.push_back(vt_nb);
                        }
                    }
                    // std::cout << "The vertices need to be updated for vehicle " << agent->vehicle_.idx_ << std::endl;
                    for(auto&vv: vert_to_upadte){
                        // std::cout << vv->state_->id_ << ", ";
                        Vertex_t<SquareCell*>* neigb_vt = neighbor->local_graph_->GetVertexFromID(vv->state_->id_);
                        if(neigb_vt->state_->p_ == 0.0 || neigb_vt->state_->p_ == 1.0){
                            vv->state_->p_ = neigb_vt->state_->p_;
                            vv->state_->occupancy_ = neigb_vt->state_->occupancy_;
                        }
                        else{
                            double prob_p_ = vv->state_->p_;
                            double prob_q_ = 1.0 - prob_p_;
                            double current_ep_ = -prob_p_ * log10(prob_p_) - prob_q_ * log10(prob_q_);

                            double neigb_p_ = neigb_vt->state_->p_;
                            double neigb_q_ = 1.0 - neigb_vt->state_->p_;
                            double neigb_ep_ = -neigb_p_ * log10(neigb_p_) - neigb_q_ * log10(neigb_q_);

                            if(neigb_ep_ < current_ep_){
                                vv->state_->p_ = neigb_vt->state_->p_;
                                vv->state_->occupancy_ = neigb_vt->state_->occupancy_;
                            }
                        }
                    }
                }
                for(int64_t ii = 0; ii < teams->auto_team_.size(); ii++){
                    if(neighbor->iteration_neighb_(ii) > agent->iteration_neighb_(ii)){
                        agent->iteration_neighb_(ii) = neighbor->iteration_neighb_(ii);
                    }
                }      
            } 
        }
        flag_ = IPASMeasurement::MapMergeConvergence(teams);
        if(flag_ == true){break;}
    }
    for(auto&agent: teams->auto_team_){
        // Update the cost of local graph
        if(agent->vehicle_type_ == TaskType::RESCUE){
            auto edges = agent->local_graph_->GetAllEdges();
            std::vector<int64_t> dst_vertices;
            for (auto& e: edges){
                double uncertain_cost = 1.0 * (1.0 - e->dst_->state_->p_) + PENALTY_*e->dst_->state_->p_;
                e->cost_ = uncertain_cost;
                if(e->dst_->state_->p_ == 1.0){
                    dst_vertices.push_back(e->dst_->state_->id_);
                }
            }
            for(auto cc: dst_vertices){
                Vertex_t<SquareCell*>* vert = agent->local_graph_->GetVertexFromID(cc);
                std::vector<Vertex_t<SquareCell*>*> neighbs = vert->GetNeighbours();
                for(auto nn: neighbs){
                    agent->local_graph_->RemoveEdge(vert->state_,nn->state_);
                }
            }
        }
    } 
}