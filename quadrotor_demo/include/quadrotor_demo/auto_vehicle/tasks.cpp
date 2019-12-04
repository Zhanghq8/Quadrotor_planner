#include "tasks.hpp"

using namespace librav;

TasksSet::TasksSet(std::vector<Task>& tasks){
    tasks_ = {};
    for(auto& tk_: tasks){
        tasks_.push_back(tk_);
    }
}

Task TasksSet::GetTaskFromID(int64_t idx){
    for(auto&tk_: tasks_){
        if(tk_.idx_ == idx){
            return tk_;
        }
    }
}

int64_t TasksSet::GetTaskPosFromID(int64_t idx){
    for(auto tk_: tasks_){
        if(tk_.idx_ == idx){
            return tk_.pos_.front();
        }
    }
}

std::unordered_set<int64_t> TasksSet::GetHotspots() {
    std::unordered_set<int64_t> hotspots;
    for(auto tk_: tasks_){
        hotspots.insert(tk_.pos_[0]);
    }
    return hotspots;
}
