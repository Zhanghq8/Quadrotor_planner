#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

using namespace std;
class Take_off {
public:

    Take_off(ros::NodeHandle* nodehandle) {
    	// ROS_INFO("In class constructor of Take_off");
    	initSub();
    	initPub();
    }

    void initPub() {
    	// ROS_INFO("Initializing Publishers");  
		drone1_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1000);
		drone2_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone2/cmd_vel", 1000);
		drone3_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone3/cmd_vel", 1000);
    }

    void initSub() {
    	// ROS_INFO("Initializing Subscribers");  
		drone1_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1000, &Take_off::Callback1, this);
		drone2_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1000, &Take_off::Callback2, this);
		drone3_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1000, &Take_off::Callback3, this);
		stop_sub_ = nh_.subscribe("/drone1/cmd_vel", 1000, &Take_off::eventCallback,this);
    }

	void Callback1(const geometry_msgs::PoseStamped& drone_pose1) {	
		// cout << "pos1: " << drone_pose1.pose.position.z << endl;
		if (drone_pose1.pose.position.z < take_off_height + 0.3 && 
			drone_pose1.pose.position.z > take_off_height - 0.3) {
			flag1 = true;
			// cout << "Great1: " << endl;
			controlinput1.linear.z = 0;
		}
		else if (drone_pose1.pose.position.z >= take_off_height + 0.4) {
			controlinput1.linear.z = -0.5;		
		}
		else if (drone_pose1.pose.position.z <= take_off_height - 0.4) {
			controlinput1.linear.z = 0.5;
		}
		else {
			controlinput1.linear.z = 0;
			flag1 = true;
		}
		drone1_pub_.publish(controlinput1);
	}
	void Callback2(const geometry_msgs::PoseStamped& drone_pose2) {
		// cout << "pos2: " << drone_pose2.pose.position.z << endl;
		if (drone_pose2.pose.position.z < take_off_height + 0.3 && 
			drone_pose2.pose.position.z > take_off_height - 0.3) {
			flag2 = true;
			// cout << "Great2: " << endl;
			controlinput2.linear.z = 0;
		}
		else if (drone_pose2.pose.position.z >= take_off_height + 0.5) {
			controlinput2.linear.z = -0.5;		
		}
		else if (drone_pose2.pose.position.z <= take_off_height - 0.5) {
			controlinput2.linear.z = 0.5;
		}
		else {
			controlinput2.linear.z = 0;
			flag2 = true;
		}
		drone2_pub_.publish(controlinput2);
	}
	void Callback3(const geometry_msgs::PoseStamped& drone_pose3) {
		// cout << "pos3: " << drone_pose3.pose.position.z << endl;
		if (drone_pose3.pose.position.z < take_off_height + 0.3 && 
			drone_pose3.pose.position.z > take_off_height - 0.3) {
			flag3 = true;
			// cout << "Great3: " << endl;
			controlinput3.linear.z = 0;
			
		}
		else if (drone_pose3.pose.position.z >= take_off_height + 0.5) {
			controlinput3.linear.z = -0.5;		
		}
		else if (drone_pose3.pose.position.z <= take_off_height - 0.5) {
			controlinput3.linear.z = 0.5;
		}
		else {
			controlinput3.linear.z = 0;
			flag3 = true;
		}
		drone3_pub_.publish(controlinput3);
	}
	void eventCallback(const geometry_msgs::Twist& vel) {
		if ((vel.linear.x != 0 || vel.linear.y != 0) && (flag1 == true && flag2 == true && flag3 == true)) {
			ROS_INFO("Taking off finished...");
            ros::shutdown();
		}
	}

	
private:
	double take_off_height = 8;
	bool flag1 = false;
	bool flag2 = false;
	bool flag3 = false;
	ros::NodeHandle nh_;
	ros::Publisher drone1_pub_;
	ros::Publisher drone2_pub_;
	ros::Publisher drone3_pub_;
	ros::Subscriber drone1_sub_;
	ros::Subscriber drone2_sub_;
	ros::Subscriber drone3_sub_;
	ros::Subscriber stop_sub_;
	geometry_msgs::Twist controlinput1;
	geometry_msgs::Twist controlinput2;
	geometry_msgs::Twist controlinput3;
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