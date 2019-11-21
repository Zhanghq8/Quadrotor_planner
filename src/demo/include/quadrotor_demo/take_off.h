#ifndef TAKE_OFF_H
#define TAKE_OFF_H

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"

class Take_off{

private:

	ros::NodeHandle nh_;
    ros::Subscriber currentpos_sub_;
    ros::Subscriber stop_sub_;
    ros::Subscriber laserpos_sub_;

public:
	Take_off(ros::NodeHandle* nodehandle);
    void initSub(); 
    void initPub();
};


#endif //TAKE_OFF_H