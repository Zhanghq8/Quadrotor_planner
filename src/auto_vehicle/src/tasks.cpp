#include "auto_vehicle/tasks.hpp"

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
