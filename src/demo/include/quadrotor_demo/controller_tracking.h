#ifndef CONTROLLER_TRACKING_H_
#define CONTROLLER_TRACKING_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Bool.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
// #include <sensor_msgs/LaserScan.h>

using namespace std;

class Controller_tracking
{
private:
    int drone_num;
    // max control input for linear velocity
    double v; 
    bool start_flag = true;
    bool ready_flag_1 = false;
    bool ready_flag_2 = false;
    bool ready_flag_3 = false;
    bool ready_flag = false;

    // pid gain parameters
    double kp;
    double kd;
    double ki;
    double d=1.0;

    vector<int> waypoint_cnt;

    struct Vec2i
    {
        float x, y;
    };

    // error dynamics, accumulated error, previous error
    //e_k, e_P, e_I, e_D, E_k, e_k_previous
    vector<vector<double>> errorvector;

    //x, y, z,>pos, orientation< x, y, z, w
    vector<vector<double>> posvector;

    vector<vector<Vec2i>> path;

    vector<vector<double>> goalposvector;

    // difference between currentpos and goalpos(x,y), and heading angle
    vector<vector<double>> diffvector;

    vector<vector<double>> xyvelocity;

    geometry_msgs::Twist control1input;
    geometry_msgs::Twist control2input;
    geometry_msgs::Twist control3input;

    ros::NodeHandle nh_;
    ros::Subscriber current1pos_sub_;
    ros::Subscriber current2pos_sub_;
    ros::Subscriber current3pos_sub_;
    // ros::Subscriber goalpos_sub_;
    ros::Subscriber stop_sub_;
    ros::Publisher control1input_pub_;
    ros::Publisher control2input_pub_;
    ros::Publisher control3input_pub_;
    ros::Publisher ready_flag_pub_;


public:
    Controller_tracking(ros::NodeHandle* nodehandle);

    void setgoalpos();
    void setwaypoint_cnt();
    void initSub(); 
    void initPub();
    void initVec();
    // void setreadyflag();

    void setpidgains(double p=0.5, double i=0.00, double d=0.0);
    void setvelocity(double x=0.8);
    void setdronenum(int x=3);
    void setpath();
    
    // void goalCallback(const geometry_msgs::PoseArray& goal);
    void currentpos1Callback(const geometry_msgs::PoseStamped& odom1);
    void currentpos2Callback(const geometry_msgs::PoseStamped& odom2);
    void currentpos3Callback(const geometry_msgs::PoseStamped& odom3);
    void eventCallback(const geometry_msgs::PoseStamped& odom1); 

};

#endif
