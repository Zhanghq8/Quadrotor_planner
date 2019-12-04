#include "../include/quadrotor_demo/pose_pub_test.h"

PosePubTest::PosePubTest(ros::NodeHandle* nodehandle, vector<vector<double>> input_path):nh_(*nodehandle), path(input_path) { 
// constructor
    ROS_INFO("In class constructor of PosePubTest");
    initVec();
    initSub();
    initPub();
}

void PosePubTest::initSub() {
    // ROS_INFO("Initializing Subscribers");  
    stop_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &PosePubTest::eventCallback,this);
}

void PosePubTest::initVec() {
    currentPose = vector<double> (vector<double> (7, 0));
    goalPose = vector<double> (vector<double> (7, 0));
}

void PosePubTest::initPub() {
    // ROS_INFO("Initializing Publishers");
    goalpos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/drone1/command/pose", 1, true); 
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
}

void PosePubTest::eventCallback(const geometry_msgs::PoseStamped& odom1) {
 	
    currentPose[0] = odom1.pose.position.x;
    currentPose[1] = odom1.pose.position.y;
    currentPose[2] = odom1.pose.position.z;
    // currentPose[3] = odom3.pose.orientation.x; 
    // currentPose[4] = odom3.pose.orientation.y;
    // currentPose[5] = odom3.pose.orientation.z;
    // currentPose[6] = odom3.pose.orientation.w;
    goalPose[0] = path[path_index][0];
    goalPose[1] = path[path_index][1];
    goalPose[2] = path[path_index][2];
    // cout << "enter" << endl;

    int pathsize = path.size();
    // cout << "Xinit pose: " << currentPose[0] << " Yinit pose: " << currentPose[1] << " Zinit pose: " << currentPose[2] <<endl;
    // cout << "Xtarget pose: " << goalPose[0] << " Ytarget pose: " << goalPose[1] << " Ztarget pose: " << goalPose[2] <<endl;
    if ((abs(currentPose[0] - goalPose[0])<reachedCheck) && (abs(currentPose[1] - goalPose[1])<reachedCheck ) 
        && (abs(currentPose[2] - goalPose[2])<reachedCheck) && (currentPose[0] * goalPose[0] > 0.01)) {   
        
        if (path_index == pathsize-1) {
        	// cout << "?" << endl;
            controlinput.linear.x = 0.0;
            controlinput.linear.y = 0.0;
            controlinput.linear.z = 0.0;
            controlinput_pub_.publish(controlinput);
            // ROS_INFO("Finished...");
            // ros::shutdown();
        } else {   
            reached = true;
    	}

    } else {
    	if (reached) {
    		path_index++;
	    	goalPose[0] = path[path_index][0];
	        goalPose[1] = path[path_index][1];
	        goalPose[2] = path[path_index][2];
	        // goalPose[3] = path[path_index-1][3];
	        // goalPose[4] = path[path_index-1][4];
	        // goalPose[5] = path[path_index-1][5];
	        // goalPose[6] = path[path_index-1][6];
	     	goal_pose.header.frame_id="world";
			// goal_pose.header.stamp = ros::Time::now();
		    goal_pose.pose.position.x = goalPose[0];
		    goal_pose.pose.position.y = goalPose[1];
		    goal_pose.pose.position.z = goalPose[2];
		    goal_pose.pose.orientation.x = goalPose[3];
		    goal_pose.pose.orientation.y = goalPose[4];
		    goal_pose.pose.orientation.z = goalPose[5];
		    goal_pose.pose.orientation.w = goalPose[6];
		    goalpos_pub_.publish(goal_pose);
		    reached = false;
    	}

    }
}

PosePubTest::~PosePubTest()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_pub"); //node name
    vector<vector<double>> path{{0,0,0,0,0,0,1},{1.5,0,0.8,0,0,0,1},{2.5,3.0,0.8,0,0,0,1},{4.5,6.0,0.8,0,0,0,1}};
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    PosePubTest posepub(&nh, path);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing posepub...");
    ros::spin();
    return 0;
}



