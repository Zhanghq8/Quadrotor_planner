#ifndef TASK_PLAN_H_
#define TASK_PLAN_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/MarkerArray.h>  
#include "std_msgs/Bool.h"
#include "hungarian.h"

using namespace std;

class Task_plan
{
private:
	int drone_num;
	bool flag = false;
    ros::NodeHandle nh_;
    vector<vector<double>> interestedpoints;
    vector<vector<double>> posvector;
    vector<int> task_index;

    ros::Publisher task_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber pos1_sub_;
    ros::Subscriber pos2_sub_;
    ros::Subscriber pos3_sub_;
    // make sure drones are ready to get new interested point
    ros::Subscriber readyflag_sub_;


public:
    Task_plan(ros::NodeHandle* nodehandle);
    void setdronenum(int x=3.0);
    void initSub(); 
    void initPub();
    void initVec();
    void generateip();   
    void plan(); 
    double distance(double x1, double y1, double x2, double y2);

    void pos1Callback(const geometry_msgs::PoseStamped& odom1);
    void pos2Callback(const geometry_msgs::PoseStamped& odom2);
    void pos3Callback(const geometry_msgs::PoseStamped& odom3);
    void flagCallback(const std_msgs::Bool& flag);

    // void currentpos1Callback(const geometry_msgs::PoseStamped& odom1);

};

#endif
