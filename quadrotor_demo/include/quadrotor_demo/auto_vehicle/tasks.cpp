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

std::vector<int64_t> TasksSet::GetHotspots() {
    std::vector<int64_t> hotspots;
    // for (int i = 0; i < tasks_.size(); i++) {
    //     std::cout << tasks_[i].pos_[0] << " !!! " << i << std::endl;
    // }
    for(auto tk_: tasks_){
        hotspots.push_back(tk_.pos_[0]);
    }

    return hotspots;
}
