#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

using namespace std;
class Take_off {
public:

    Take_off(ros::NodeHandle* nodehandle) {
    	// ROS_INFO("In class constructor of Take_off");
    	initPub();
    	initSub();
    }

    void initPub() {
    	// ROS_INFO("Initializing Publishers");  
		drone1_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1000);
    }

    void initSub() {
    	// ROS_INFO("Initializing Subscribers");  
		drone1_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1000, &Take_off::Callback1, this);
    }

	void Callback1(const geometry_msgs::PoseStamped& drone_pose1) {	
		// cout << "pos1: " << drone_pose1.pose.position.z << endl;
		// if (drone_pose1.pose.position.z < take_off_height + 0.3 && 
		// 	drone_pose1.pose.position.z > take_off_height - 0.3) {
		// 	flag1 = true;
		// 	// cout << "Great1: " << endl;
		// 	controlinput1.linear.z = 0;
		// }
		// else if (drone_pose1.pose.position.z >= take_off_height + 0.4) {
		// 	controlinput1.linear.z = -0.5;		
		// }
		// else if (drone_pose1.pose.position.z <= take_off_height - 0.4) {
		// 	controlinput1.linear.z = 0.5;
		// }
		// else {
		// 	controlinput1.linear.z = 0;
		// 	flag1 = true;
		// }

		controlinput1.linear.z = 0.5;
		int i=0;
		while (i < 5) {
			drone1_pub_.publish(controlinput1);
			i++;
			cout << "acc" << i << endl;
			loop_rate.sleep();
		}
		while (flag1 == false) {
			controlinput1.linear.z = 0.0;
			cout << "after" << endl;
			drone1_pub_.publish(controlinput1);
		}

	}
	void eventCallback(const geometry_msgs::Twist& vel) {
		if ((vel.linear.x != 0 || vel.linear.y != 0) && flag1 == false) {
			ROS_INFO("Taking off finished...");
            ros::shutdown();
		}
	}
	
	
private:
	bool flag1 = false;
	ros::NodeHandle nh_;
	ros::Publisher drone1_pub_;
	ros::Subscriber drone1_sub_;
	ros::Subscriber stop_sub_;
	ros::Rate loop_rate{3.0};
	geometry_msgs::Twist controlinput1;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "take_off");

	ros::NodeHandle nh;
    Take_off take_off(&nh); 
    ROS_INFO("Taking off...");
    ros::spin();

	return 0;
}