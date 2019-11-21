#include "../include/quadrotor_demo/controller.h"

Controller::Controller(ros::NodeHandle* nodehandle):nh_(*nodehandle) { 
// constructor
    // ROS_INFO("In class constructor of Controller");

    setdronenum();
    setvelocity();
    setpidgains();
    initVec();
    initSub();
    initPub();
}

void Controller::initVec() {
    errorvector = vector<vector<double>>(drone_num, vector<double> (6, 0));
    posvector = vector<vector<double>> (drone_num, vector<double> (7, 0));
    goalposvector = vector<vector<double>> (drone_num, vector<double> (7, 0));

    diffvector = vector<vector<double>> (drone_num, vector<double> (3, 0));
    xyvelocity = vector<vector<double>> (drone_num, vector<double> (3, 0));
}

void Controller::setpidgains(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
}

void Controller::setvelocity(double x) {
    v = x;
}

void Controller::setdronenum(int num) {
    drone_num = num;
}

void Controller::initSub() {
    // ROS_INFO("Initializing Subscribers");  

    current1pos_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &Controller::currentpos1Callback,this);
    current2pos_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &Controller::currentpos2Callback,this);
    current3pos_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &Controller::currentpos3Callback,this);
    goalpos_sub_ = nh_.subscribe("/pose_pub", 1, &Controller::goalCallback,this); 
}

//member helper function to set up publishers;
void Controller::initPub() {
    // ROS_INFO("Initializing Publishers");
    control1input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
    control2input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone2/cmd_vel", 1, true);
    control3input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone3/cmd_vel", 1, true);
    ready_flag_pub_ = nh_.advertise<std_msgs::Bool>("/ready_flag", 1, true);
}

void Controller::goalCallback(const geometry_msgs::PoseArray& goal) {
    start_flag = true;
    for (int i=0; i<drone_num; i++) {
        for (int j=0; j<2; j++) {
            if (j == 0) {
                goalposvector[i][j] = goal.poses[i].position.x;
            }
            if (j == 1) {
                goalposvector[i][j] = goal.poses[i].position.y;
            } 
        }
    }
}

void Controller::currentpos1Callback(const geometry_msgs::PoseStamped& odom1) {
    posvector[0][0] = odom1.pose.position.x;
    posvector[0][1] = odom1.pose.position.y;
    posvector[0][2] = odom1.pose.position.z;
    posvector[0][3] = odom1.pose.orientation.x; 
    posvector[0][4] = odom1.pose.orientation.y;
    posvector[0][5] = odom1.pose.orientation.z;
    posvector[0][6] = odom1.pose.orientation.w;
}

void Controller::currentpos2Callback(const geometry_msgs::PoseStamped& odom2) {
    posvector[1][0] = odom2.pose.position.x;
    posvector[1][1] = odom2.pose.position.y;
    posvector[1][2] = odom2.pose.position.z;
    posvector[1][3] = odom2.pose.orientation.x; 
    posvector[1][4] = odom2.pose.orientation.y;
    posvector[1][5] = odom2.pose.orientation.z;
    posvector[1][6] = odom2.pose.orientation.w;
}

void Controller::currentpos3Callback(const geometry_msgs::PoseStamped& odom3) {   

    posvector[2][0] = odom3.pose.position.x;
    posvector[2][1] = odom3.pose.position.y;
    posvector[2][2] = odom3.pose.position.z;
    posvector[2][3] = odom3.pose.orientation.x; 
    posvector[2][4] = odom3.pose.orientation.y;
    posvector[2][5] = odom3.pose.orientation.z;
    posvector[2][6] = odom3.pose.orientation.w;

    double pose_error = 0;
    // if (start_flag == true && ready_flag == true) {
    for (int i=0; i<drone_num; i++) {
        for (int j=0; j<diffvector[i].size(); j++) {
            // cout << i << ", " << j << " " << goalposvector[i][j] << " " << posvector[i][j] << endl;
            if (j == int(diffvector[i].size()) - 1) {
                diffvector[i][j] = atan2(diffvector[i][j-1], diffvector[i][j-2]);
                pose_error += sqrt(diffvector[i][j-1]*diffvector[i][j-1] + diffvector[i][j-2]*diffvector[i][j-2]);
            }
            else {
                diffvector[i][j] = goalposvector[i][j] - posvector[i][j];
            }
        }
    }
    if (pose_error < 0.05 || (goalposvector[0][0] == 0 && goalposvector[drone_num-1][1] == 0)) {
        ready_flag = true;
    }
    else {
        ready_flag = false;
    }
    // cout << "flag: " << ready_flag << endl;
    // cout << "pose error: " << pose_error << endl;
    // cout << "goal pose: " << goalposvector[0][0] << endl;
    std_msgs::Bool flag;
    flag.data = ready_flag;
    ready_flag_pub_.publish(flag);

    if (start_flag == true) {
        //e_k, e_P, e_I, e_D, E_k, e_k_previous 
        for (int i=0; i<drone_num; i++) {
            errorvector[i][0] = sqrt(diffvector[i][0]*diffvector[i][0] + diffvector[i][1]*diffvector[i][1]);
            // e_P = e_k;
            errorvector[i][1] = errorvector[i][0];
            // e_I = e_k + E_k;
            errorvector[i][2] = errorvector[i][1] + errorvector[i][4];
            //e_D = e_k - e_k_previous; 
            errorvector[i][3] = errorvector[i][0] - errorvector[i][5];
            // E_k = e_I;
            errorvector[i][4] = errorvector[i][2];
            // e_k_previous = e_k;
            errorvector[i][5] = errorvector[i][0];
        }
        // v = k_p*e_P + k_i*e_I + k_d*e_D;
        for (int i=0; i<drone_num; i++) {
            xyvelocity[i][0] = kp*errorvector[i][1] + ki*errorvector[i][2] + kd*errorvector[i][3];
            if (xyvelocity[i][0] > v) {
                xyvelocity[i][0] = v;
            }
            xyvelocity[i][1] = xyvelocity[i][0] * cos(diffvector[i][diffvector[i].size()-1]);
            xyvelocity[i][2] = xyvelocity[i][0] * sin(diffvector[i][diffvector[i].size()-1]);
            // cout << "drone " << i << ": vx, " << xyvelocity[i][1];
            // cout << "drone " << i << ": vy, " << xyvelocity[i][2];
        }

        // drone1
        control1input.linear.x = xyvelocity[0][1];
        control1input.linear.y = xyvelocity[0][2];
        control1input.linear.z = 0.0;
        control1input_pub_.publish(control1input);
        // drone2
        control2input.linear.x = xyvelocity[1][1];
        control2input.linear.y = xyvelocity[1][2];
        control2input.linear.z = 0.0;
        control2input_pub_.publish(control2input);
        // drone3
        control3input.linear.x = xyvelocity[2][1];
        control3input.linear.y = xyvelocity[2][2];
        control3input.linear.z = 0.0;
        control3input_pub_.publish(control3input);

    }    
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "demo"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Controller controller(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing controller...");
    ros::spin();
    return 0;
}