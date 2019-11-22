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
    // local_graph_ = nullptr;
    hotspots_ = {};
    nz_ig_zone_ = {};
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
void AutoVehicle::UpdateReward(TasksSet tasks,std::shared_ptr<Graph_t<SquareCell*>> graph){
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
            double path_cost = PathCostComputation(tasks,graph,dup_path_[mm],pos_);
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
    if (tk_path.empty()){
        return 0.0;
    }
    else{
        Task final_tk = tasks.GetTaskFromID(tk_path.back());
        Path_t<SquareCell*> path;

        int64_t start_idx = init_pos;
        for(auto& tk_idx: tk_path){
            Task task = tasks.GetTaskFromID(tk_idx);
            // Whether vehicle is able to acomplish the task
            if(vehicle_type_ == task.task_type_){
                for(auto& pos: task.pos_){
                    // Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(pos);
                    Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(pos);
                    int64_t target_idx = vert->state_->id_;
                    Path_t<SquareCell*> path_seg = {};
                    // Path_t<SquareCell*> path_seg = AStar::Search(graph,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristicUncertain));
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
            else{
                return NOT_CAPABILITY_COST_;
            }
        }
        
        
        // path_cost_ = double(path.size());
        if(vehicle_type_ == TaskType::RESCUE){
            for(int64_t kk = 0; kk < path.size()-1; kk++){
                path_cost_ += GridGraph::CalcHeuristicUncertain(path[kk],path[kk+1]);
            }
        }
        else if (vehicle_type_ == TaskType::MEASURE){
            path_cost_ = double(path.size());
        }
        return path_cost_;
    }
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
    
void AutoVehicle::BundleAdd(TasksSet tasks,std::shared_ptr<Graph_t<SquareCell*>> graph){
    while(task_bundle_.size() <= MAX_TASKS){
        // Update the reward 
        UpdateReward(tasks,graph);
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
Path_t<SquareCell*> AutoVehicle::PathComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> true_graph){
    Path_t<SquareCell*> path;
    if (task_path_.empty()){
        return path;
    }
    else{
        Task final_tk = tasks.GetTaskFromID(task_path_.back());
        int64_t start_idx = pos_;
        for(auto& tk_idx: task_path_){
            Task task = tasks.GetTaskFromID(tk_idx);
            // Whether vehicle is able to acomplish the task
            if(vehicle_type_ == task.task_type_){
                for(auto& pos: task.pos_){
                    Vertex_t<SquareCell*>* vert = true_graph->GetVertexFromID(pos);
                    int64_t target_idx = vert->state_->id_;
                    Path_t<SquareCell*> path_seg;
                    if(vehicle_type_ == TaskType::RESCUE){
                        path_seg = AStar::Search(true_graph,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristicUncertain));
                    }
                    else if (vehicle_type_ == TaskType::MEASURE){
                        path_seg = AStar::SearchIgnoreEdgeCost(true_graph,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
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

std::map<int64_t,Path_t<SquareCell*>> IPASMeasurement::GeneratePaths(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,TasksSet tasks,std::shared_ptr<Graph_t<SquareCell*>> graph, TaskType task_type){
    std::map<int64_t,Path_t<SquareCell*>> paths_map_;
    for(auto agent: vehicle_team->auto_team_){
        if (agent->vehicle_type_ == task_type){
            if(!agent->task_path_.empty()){
                paths_map_[agent->idx_] = agent->PathComputation(tasks,graph);
            }
            else{
                Path_t<SquareCell*> empty_path_ = {};
                paths_map_[agent->idx_] = empty_path_;
            }
        }    
    }
    return paths_map_;
}

