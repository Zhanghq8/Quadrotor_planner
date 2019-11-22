/*
 * tasks.hpp
 *
 *  Created on: July 23, 2019
 *      Author: jfang
 */

#ifndef TASKS_HPP
#define TASKS_HPP

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>

#include "../../../graph/graph.hpp"
#include "../../../graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"

namespace librav{
    enum class TaskType{
        MEASURE,
        RESCUE
    };

    class Task
    {
        public:
            Task(int64_t idx,int8_t ap_idx,std::vector<int64_t> pos,TaskType tp,int64_t num_v): 
                idx_(idx),AP_label_(ap_idx),pos_(pos),task_type_(tp),num_vehicles_(num_v){};
            ~Task(){};

            int64_t idx_;
            int8_t AP_label_;
            std::vector<int64_t> pos_;
            TaskType task_type_;
            int64_t num_vehicles_;
    };

    class TasksSet
    {
        public:
            TasksSet(std::vector<Task>& tasks);
            TasksSet(){};
            // TasksSet(LTLFormula Global_LTL)
            ~TasksSet(){};

            std::vector<Task> tasks_;

            Task GetTaskFromID(int64_t idx);
            int64_t GetTaskPosFromID(int64_t idx);
    };
}

#endif /* TASKS_HPP */