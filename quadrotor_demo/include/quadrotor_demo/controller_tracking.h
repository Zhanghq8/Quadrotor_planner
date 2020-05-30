#ifndef CONTROLLER_TRACKING_H_
#define CONTROLLER_TRACKING_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <bitset>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Bool.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadrotor_demo/pose.h"
#include "quadrotor_demo/path.h"
#include "quadrotor_demo/pathes.h"
#include "quadrotor_demo/final_path.h"

using namespace std;

class Controller_tracking
{
private:
    int drone_num;
    // max control input for linear velocity
    double v; 
    bool start_flag = false;
    // bool ready_flag_1 = false;
    // bool ready_flag_2 = false;
    // bool ready_flag_3 = false;
    // bool ready_flag = false;

    vector<bool> updateComplete;

    vector<bool> pidFlag;

    vector<bool> switchFlag01;
    vector<bool> switchFlag12;
    vector<bool> switchFlag23;
    vector<bool> switchFlag34;
    vector<bool> updateMapFlag;

    bool switchFlag0 = false;
    bool switchFlag1 = false;
    bool switchFlag2 = false;
    bool switchFlag3 = false;
    // pid gain parameters
    double kp;
    double kd;
    double ki;
    double d=0.5;

    vector<int> waypoint_cnt;
    // vector<int> waypoint_cnt2;
    // vector<int> waypoint_cnt3;

    struct Vec2i
    {   
        int64_t id;
        float x, y, z;
        int64_t xcoord, ycoord;
    };

    // error dynamics, accumulated error, previous error
    //e_k, e_P, e_I, e_D, E_k, e_k_previous
    vector<vector<double>> errorvector;

    //x, y, z,>pos, orientation< x, y, z, w
    vector<vector<double>> posvector;

    // task
    vector<vector<vector<Vec2i>>> sensorPath;

    vector<vector<double>> goalposvector;

    // difference between currentpos and goalpos(x,y), and heading angle
    vector<vector<double>> diffvector;

    vector<vector<double>> xyvelocity;

    geometry_msgs::Twist control1input;
    geometry_msgs::Twist control2input;
    geometry_msgs::Twist control3input;
    geometry_msgs::Twist control4input;

    ros::NodeHandle nh_;
    ros::Subscriber current1pos_sub_;
    ros::Subscriber current2pos_sub_;
    ros::Subscriber current3pos_sub_;
    ros::Subscriber current4pos_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber event1_sub_;
    ros::Subscriber event2_sub_;
    ros::Subscriber event3_sub_;
    ros::Subscriber event4_sub_;
    ros::Subscriber update1_complete_sub_;
    ros::Subscriber update2_complete_sub_;
    ros::Subscriber update3_complete_sub_;
    ros::Subscriber update4_complete_sub_;

    ros::Publisher control1input_pub_;
    ros::Publisher control2input_pub_;
    ros::Publisher control3input_pub_;
    ros::Publisher control4input_pub_;
    ros::Publisher ready_flag_pub_;
    ros::Publisher update1Map_flag_pub_;
    ros::Publisher update2Map_flag_pub_;
    ros::Publisher update3Map_flag_pub_;
    ros::Publisher update4Map_flag_pub_;
    ros::Publisher updategraph_flag_pub_;


public:
    Controller_tracking(ros::NodeHandle* nodehandle);

    void setgoalpos();
    void initSub(); 
    void initPub();
    void initVec();
    // void setreadyflag();

    void setpidgains(double p=0.5, double i=0.00, double d=0.0);
    void setvelocity(double x=0.6);
    void setdronenum(int x=4);
    void updateMap();
    // void setpath();
    
    // void goalCallback(const geometry_msgs::PoseArray& goal);
    void currentpos1Callback(const geometry_msgs::PoseStamped& odom1);
    void currentpos2Callback(const geometry_msgs::PoseStamped& odom2);
    void currentpos3Callback(const geometry_msgs::PoseStamped& odom3);
    void currentpos4Callback(const geometry_msgs::PoseStamped& odom4);
    void event1Callback(const geometry_msgs::PoseStamped& odom1);
    void event2Callback(const geometry_msgs::PoseStamped& odom2);
    void event3Callback(const geometry_msgs::PoseStamped& odom3); 
    void event4Callback(const geometry_msgs::PoseStamped& odom4); 
    void pathCallback(const quadrotor_demo::final_path& path);
    void update1CompleteCallback(const std_msgs::Bool& flag1);
    void update2CompleteCallback(const std_msgs::Bool& flag2);
    void update3CompleteCallback(const std_msgs::Bool& flag3);
    void update4CompleteCallback(const std_msgs::Bool& flag4);

};

#endif
