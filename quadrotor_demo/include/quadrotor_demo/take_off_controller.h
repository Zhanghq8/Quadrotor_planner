#ifndef TAKE_OFF_CONTROLLER_H_
#define TAKE_OFF_CONTROLLER_H_

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

class Takeoff_Controller
{
private:
    int drone_num;
    // max control input for linear velocity
    double v;
    double w; 
    bool start_flag = true;
    bool flag1 = false;
    bool flag2 = false;
    bool flag3 = false;
    // bool ready_flag = true;

    // pid gain parameters
    double kp;
    double kd;
    double ki;

    // error dynamics, accumulated error, previous error
    //e_k, e_P, e_I, e_D, E_k, e_k_previous
    vector<vector<double>> errorvector;

    //x, y, z,>pos, orientation< x, y, z, w
    vector<vector<double>> posvector;

    vector<vector<double>> goalposvector;

    // difference between currentpos and goalpos(z), and yaw
    vector<vector<double>> diffvector;

    vector<vector<double>> zyawvelocity;

    // geometry_msgs::PoseStamped current1pos, goal1pos;
    // geometry_msgs::PoseStamped current2pos, goal2pos;
    // geometry_msgs::PoseStamped current3pos, goal3pos;
    geometry_msgs::Twist control1input;
    geometry_msgs::Twist control2input;
    geometry_msgs::Twist control3input;

    ros::NodeHandle nh_;
    ros::Subscriber current1pos_sub_;
    ros::Subscriber current2pos_sub_;
    ros::Subscriber current3pos_sub_;
    ros::Subscriber stop_sub_;
    // ros::Subscriber laserpos_sub_;
    ros::Publisher control1input_pub_;
    ros::Publisher control2input_pub_;
    ros::Publisher control3input_pub_;
    // ros::Publisher ready_flag_pub_;

public:
    Takeoff_Controller(ros::NodeHandle* nodehandle);
    void initSub(); 
    void initPub();
    void initVec();

    // void setgoalpos(double x, double y);
    void setpidgains(double p=1.0, double i=0.00, double d=0.2);
    void setvelocity(double x=1.0, double yaw=0.05);
    void setdronenum(int x=3);
    
    void setgoal(double z=6.0, double yaw=0.0);
    void currentpos1Callback(const geometry_msgs::PoseStamped& odom1);
    void currentpos2Callback(const geometry_msgs::PoseStamped& odom2);
    void currentpos3Callback(const geometry_msgs::PoseStamped& odom3);
    void eventCallback(const geometry_msgs::Twist& vel); 

};

#endif