#ifndef POSE_PUB_H_
#define POSE_PUB_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
// #include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseArray.h"

#include <geometry_msgs/Pose2D.h>

using namespace std;

class Pose_pub
{
private:

    ros::NodeHandle nh_;
    // ros::Subscriber current1pos_sub_;

    ros::Publisher goalpos_pub_;

public:
    Pose_pub(ros::NodeHandle* nodehandle);
    // void initSub(); 
    void initPub();    

    // void currentpos1Callback(const geometry_msgs::PoseStamped& odom1);

};

#endif
