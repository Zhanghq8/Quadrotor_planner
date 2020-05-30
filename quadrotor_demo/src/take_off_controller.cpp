#include "../include/quadrotor_demo/take_off_controller.h"

Takeoff_Controller::Takeoff_Controller(ros::NodeHandle* nodehandle):nh_(*nodehandle) { 
// constructor
    // ROS_INFO("In class constructor of Takeoff_Controller");

    setdronenum();
    initVec();
    setgoal();
    setvelocity();
    setpidgains();
    initSub();
    initPub();
}

void Takeoff_Controller::initVec() {
    errorvector = vector<vector<double>>(drone_num, vector<double> (12, 0));
    posvector = vector<vector<double>> (drone_num, vector<double> (7, 0));
    goalposvector = vector<vector<double>> (drone_num, vector<double> (7, 0));

    diffvector = vector<vector<double>> (drone_num, vector<double> (2, 0));
    zyawvelocity = vector<vector<double>> (drone_num, vector<double> (2, 0));
}

void Takeoff_Controller::setpidgains(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
}

void Takeoff_Controller::setvelocity(double x, double yaw) {
    v = x;
    w = yaw;
}

void Takeoff_Controller::setdronenum(int num) {
    drone_num = num;
}

void Takeoff_Controller::initSub() {
    // ROS_INFO("Initializing Subscribers");  

    current1pos_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &Takeoff_Controller::currentpos1Callback,this);
    current2pos_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &Takeoff_Controller::currentpos2Callback,this);
    current3pos_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &Takeoff_Controller::currentpos3Callback,this);
    current4pos_sub_ = nh_.subscribe("/drone4/ground_truth_to_tf/pose", 1, &Takeoff_Controller::currentpos4Callback,this);
    stop_sub_ = nh_.subscribe("/drone1/cmd_vel", 1000, &Takeoff_Controller::eventCallback,this);
}

//member helper function to set up publishers;
void Takeoff_Controller::initPub() {
    // ROS_INFO("Initializing Publishers");
    control1input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
    control2input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone2/cmd_vel", 1, true);
    control3input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone3/cmd_vel", 1, true);
    control4input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone4/cmd_vel", 1, true);
    // ready_flag_pub_ = nh_.advertise<std_msgs::Bool>("/ready_flag", 1, true);
}

void Takeoff_Controller::setgoal(double z, double yaw) {
    // start_flag = true;
    for (int i=0; i<drone_num; i++) {
        //j = 2->z, j=6->yaw
        goalposvector[i][2] = z;

        goalposvector[i][5] = yaw;
    }
}

void Takeoff_Controller::currentpos1Callback(const geometry_msgs::PoseStamped& odom1) {
    posvector[0][0] = odom1.pose.position.x;
    posvector[0][1] = odom1.pose.position.y;
    posvector[0][2] = odom1.pose.position.z;
    posvector[0][3] = odom1.pose.orientation.x; 
    posvector[0][4] = odom1.pose.orientation.y;
    posvector[0][5] = odom1.pose.orientation.z;
    posvector[0][6] = odom1.pose.orientation.w;
}

void Takeoff_Controller::currentpos2Callback(const geometry_msgs::PoseStamped& odom2) {
    posvector[1][0] = odom2.pose.position.x;
    posvector[1][1] = odom2.pose.position.y;
    posvector[1][2] = odom2.pose.position.z;
    posvector[1][3] = odom2.pose.orientation.x; 
    posvector[1][4] = odom2.pose.orientation.y;
    posvector[1][5] = odom2.pose.orientation.z;
    posvector[1][6] = odom2.pose.orientation.w;
}

void Takeoff_Controller::currentpos3Callback(const geometry_msgs::PoseStamped& odom3) {
    posvector[2][0] = odom3.pose.position.x;
    posvector[2][1] = odom3.pose.position.y;
    posvector[2][2] = odom3.pose.position.z;
    posvector[2][3] = odom3.pose.orientation.x; 
    posvector[2][4] = odom3.pose.orientation.y;
    posvector[2][5] = odom3.pose.orientation.z;
    posvector[2][6] = odom3.pose.orientation.w;
}

void Takeoff_Controller::currentpos4Callback(const geometry_msgs::PoseStamped& odom4) {   

    posvector[3][0] = odom4.pose.position.x;
    posvector[3][1] = odom4.pose.position.y;
    posvector[3][2] = odom4.pose.position.z;
    posvector[3][3] = odom4.pose.orientation.x; 
    posvector[3][4] = odom4.pose.orientation.y;
    posvector[3][5] = odom4.pose.orientation.z;
    posvector[3][6] = odom4.pose.orientation.w;

    // double pose_error = 0;
    // if (start_flag == true && ready_flag == true) {
    for (int i=0; i<drone_num; i++) {
        for (int j=0; j<diffvector[i].size(); j++) {
            // cout << i << ", " << j << " " << goalposvector[i][j] << " " << posvector[i][j] << endl;
            if (j == 0) {
                diffvector[i][j] = goalposvector[i][2] - posvector[i][2];
                // pose_error += sqrt(diffvector[i][0]*diffvector[i][0] + diffvector[i][1]*diffvector[i][1]);
            }
            else {
                diffvector[i][j] = goalposvector[i][6] - posvector[i][5];
            }
        }
    }

    if (start_flag == true) {
        //e_k, e_P, e_I, e_D, E_k, e_k_previous 
        for (int i=0; i<drone_num; i++) {

            //z
            errorvector[i][0] = diffvector[i][0];
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

            //yaw

            errorvector[i][6] = diffvector[i][1];
            // e_P = e_k;
            errorvector[i][7] = errorvector[i][6];
            // e_I = e_k + E_k;
            errorvector[i][8] = errorvector[i][7] + errorvector[i][10];
            //e_D = e_k - e_k_previous; 
            errorvector[i][9] = errorvector[i][6] - errorvector[i][11];
            // E_k = e_I;
            errorvector[i][10] = errorvector[i][8];
            // e_k_previous = e_k;
            errorvector[i][11] = errorvector[i][6];
        }

        // v = k_p*e_P + k_i*e_I + k_d*e_D;
        for (int i=0; i<drone_num; i++) {
            zyawvelocity[i][0] = kp*errorvector[i][1] + ki*errorvector[i][2] + kd*errorvector[i][3];
            zyawvelocity[i][1] = kp*errorvector[i][7] + ki*errorvector[i][8] + kd*errorvector[i][9];
            if (zyawvelocity[i][0] >= v) {
                zyawvelocity[i][0] = v;
            }
            else if (zyawvelocity[i][0] <= -v){
                zyawvelocity[i][0] = -v;
            }
            if (zyawvelocity[i][1] >= w) {
                zyawvelocity[i][1] = w;
            }
            else if (zyawvelocity[i][1] <= -w){
                zyawvelocity[i][1] = -w;
            }
            // cout << "drone " << i << ": vx, " << zyawvelocity[i][1];
            // cout << "drone " << i << ": vy, " << zyawvelocity[i][2];
        }

        // drone1
        control1input.linear.z = zyawvelocity[0][0];
        control1input.angular.z = zyawvelocity[0][1];
        control1input_pub_.publish(control1input);
        // drone2
        control2input.linear.z = zyawvelocity[1][0];
        control2input.angular.z = zyawvelocity[1][1];
        control2input_pub_.publish(control2input);
        // drone3
        control3input.linear.z = zyawvelocity[2][0];
        control3input.angular.z = zyawvelocity[2][1];
        control3input_pub_.publish(control3input);
        // drone3
        control4input.linear.z = zyawvelocity[3][0];
        control4input.angular.z = zyawvelocity[3][1];
        control4input_pub_.publish(control4input);

        if (fabs(diffvector[0][0]) < 0.05 && fabs(diffvector[0][1]) < 0.05) {
            flag1 = true;
        }
        if (fabs(diffvector[1][0]) < 0.05 && fabs(diffvector[1][1]) < 0.05) {
            flag2 = true;
        }
        if (fabs(diffvector[2][0]) < 0.05 && fabs(diffvector[2][1]) < 0.05) {
            flag3 = true;
        }
        if (fabs(diffvector[3][0]) < 0.05 && fabs(diffvector[3][1]) < 0.05) {
            flag4 = true;
        }

    }    
}

void Takeoff_Controller::eventCallback(const geometry_msgs::Twist& vel) {
    if (flag1 == true && flag2 == true && flag3 == true && flag4 == true) {
        cnt++;
        if (cnt == 1) {
            ROS_INFO("Taking off finished...");
        }
    }
    if ((vel.linear.x != 0 || vel.linear.y != 0) && (flag1 == true && flag2 == true && flag3 == true && flag4 == true)) {
        ROS_INFO("Taking off node shutdown...");
        ros::shutdown();
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "take_off"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Takeoff_Controller takeoff_controller(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing takeoff_controller...");
    ros::spin();
    return 0;
}