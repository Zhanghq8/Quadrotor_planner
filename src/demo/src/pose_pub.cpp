#include "../include/quadrotor_demo/pose_pub.h"

Pose_pub::Pose_pub(ros::NodeHandle* nodehandle):nh_(*nodehandle) { 
// constructor
    // ROS_INFO("In class constructor of Pose_pub");

    // initSub();
    initPub();
}

// void Controller::initSub() {
//     ROS_INFO("Initializing Subscribers");  

//     current1pos_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &Controller::currentpos1Callback,this);
//     current2pos_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &Controller::currentpos2Callback,this);
//     current3pos_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &Controller::currentpos3Callback,this);
// }

//member helper function to set up publishers;
void Pose_pub::initPub() {
    // ROS_INFO("Initializing Publishers");
    // goalpos_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/sensor_plan", 1, true); 
    goalpos_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/pose_pub", 1, true); 
}

// void Controller::currentpos3Callback(const geometry_msgs::PoseStamped& odom3) {   

//     posvector[2][0] = odom3.pose.position.x;
//     posvector[2][1] = odom3.pose.position.y;
//     posvector[2][2] = odom3.pose.position.z;
//     posvector[2][3] = odom3.pose.orientation.x; 
//     posvector[2][4] = odom3.pose.orientation.y;
//     posvector[2][5] = odom3.pose.orientation.z;
//     posvector[2][6] = odom3.pose.orientation.w;

//     for (int i=0; i<drone_num; i++) {
//         for (int j=0; j<diffvector[i].size(); j++) {
//             if (j == int(diffvector[i].size()) - 1) {
//                 diffvector[i][j] = atan2(diffvector[i][j-1], diffvector[i][j-2]);
//             }
//             else {
//                 diffvector[i][j] = goalposvector[i][j] - posvector[i][j];
//             }
//         }
//     }

//     //e_k, e_P, e_I, e_D, E_k, e_k_previous 
//     for (int i=0; i<drone_num; i++) {
//         errorvector[i][0] = sqrt(diffvector[i][0]*diffvector[i][0] + diffvector[i][1]*diffvector[i][1]);
//         // e_P = e_k;
//         errorvector[i][1] = errorvector[i][0];
//         // e_I = e_k + E_k;
//         errorvector[i][2] = errorvector[i][1] + errorvector[i][4];
//         //e_D = e_k - e_k_previous; 
//         errorvector[i][3] = errorvector[i][0] - errorvector[i][5];
//         // E_k = e_I;
//         errorvector[i][4] = errorvector[i][2];
//         // e_k_previous = e_k;
//         errorvector[i][5] = errorvector[i][0];
//     }
//     // v = k_p*e_P + k_i*e_I + k_d*e_D;
//     for (int i=0; i<drone_num; i++) {
//         xyvelocity[i][0] = kp*errorvector[i][1] + ki*errorvector[i][2] + kd*errorvector[i][3];
//         if (xyvelocity[i][0] > v) {
//             xyvelocity[i][0] = v;
//         }
//         xyvelocity[i][1] = xyvelocity[i][0] * cos(diffvector[i][diffvector[i].size()-1]);
//         xyvelocity[i][2] = xyvelocity[i][0] * sin(diffvector[i][diffvector[i].size()-1]);
//         // cout << "drone " << i << ": vx, " << xyvelocity[i][1];
//         // cout << "drone " << i << ": vy, " << xyvelocity[i][2];
//     }

//     // drone1
//     control1input.linear.x = xyvelocity[0][1];
//     control1input.linear.y = xyvelocity[0][2];
//     control1input.linear.z = 0.0;
//     control1input_pub_.publish(control1input);
//     // drone2
//     control2input.linear.x = xyvelocity[1][1];
//     control2input.linear.y = xyvelocity[1][2];
//     control2input.linear.z = 0.0;
//     control2input_pub_.publish(control2input);
//     // drone3
//     control3input.linear.x = xyvelocity[2][1];
//     control3input.linear.y = xyvelocity[2][2];
//     control3input.linear.z = 0.0;
//     control3input_pub_.publish(control3input);
    
// }


int main(int argc, char** argv)
{

    ros::init(argc, argv, "pose_pub"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Pose_pub pose_pub(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing sensor goal publishing...");
    ros::spin();
    return 0;
}

// rostopic pub /pose_pub geometry_msgs/PoseArray "{header: {frame_id: 'sensor'}, poses: [{position: {x: 10.0, y: 10.0}}, {position: {x: 10, y: -10.0}}, {position: {x: -10.0, y: -10.0}}]}"
