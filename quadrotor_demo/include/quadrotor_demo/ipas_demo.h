#ifndef IPAS_DEMO_H_
#define IPAS_DEMO_H_

#include <stdio.h>
#include <iostream> 
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
#include <assert.h> 
#include <unordered_set>

// self-defined library
#include "graph/graph.hpp"
#include "graph/astar.hpp"
#include "map/square_grid.hpp"

#include "auto_vehicle/auto_vehicle.hpp"
#include "auto_vehicle/tasks.hpp"
#include "auto_team/auto_team.hpp"
#include "task_assignment/cbba_impl.hpp"

// ros library
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>  
#include <std_msgs/Bool.h>
#include "quadrotor_demo/pose.h"
#include "quadrotor_demo/path.h"
#include "quadrotor_demo/pathes.h"
#include "quadrotor_demo/final_path.h"

using namespace librav;

class IpasDemo{

private:

	// ros param
    ros::NodeHandle nh_;
    ros::Publisher task_pub_;
    // ros::Publisher marker_pub_;
    ros::Subscriber currentpos1_sub_;
    ros::Subscriber currentpos2_sub_;
    ros::Subscriber currentpos3_sub_;
    // make sure drones are ready to get new interested point
    ros::Subscriber updatemapflag_sub_;

    // ipas param
    int64_t num_vehicle_;
    int64_t num_tasks_;
    int64_t num_sensors_;

    // task for mobile robot
    std::vector<Task> tasks_data_;
    Eigen::MatrixXi comm_;
    // agent type
    std::vector<AutoVehicle> agents_;
    TasksSet tasks_;
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_;

    int64_t num_row_;
    int64_t num_col_;
    std::vector<std::vector<int64_t>> range_idx_;

    int64_t ipas_tt;
    bool updatemap_flag;
    std::unordered_set<int64_t> hotspots;

    std::shared_ptr<SquareGrid> uncertain_grid;
    std::shared_ptr<Graph_t<SquareCell*>> uncertain_graph;
    std::shared_ptr<SquareGrid> true_grid;
    std::shared_ptr<Graph_t<SquareCell *>> true_graph;

    // vector<vector<double>> interestedpoints;
    // vector<vector<double>> posvector;
    // vector<int> task_index;


public:
	// IpasDemo(ros::NodeHandle* nodehandle);
	IpasDemo(ros::NodeHandle* nodehandle, std::vector<Task>& tasks_data, std::vector<AutoVehicle>& agent, 
			Eigen::MatrixXi& comm_, int64_t num_vehicle, int64_t num_tasks, int64_t num_sensors, int64_t num_row, 
			int64_t num_col, std::vector<std::vector<int64_t>> range_idx);
	~IpasDemo();
	void init();
	void initMap();
    void initSub(); 
    void initPub();
    void mobilePath();
    void sensorPath();
    void pathesPub(const std::map<int64_t,Path_t<SquareCell*>>& pathes);
    void currentpos1Callback(const geometry_msgs::PoseStamped& odom1);
    void currentpos2Callback(const geometry_msgs::PoseStamped& odom2);
    void currentpos3Callback(const geometry_msgs::PoseStamped& odom3);
    void updatemapflagCallback(const std_msgs::Bool& flag);
};

#endif /* IPAS_DEMO_H */