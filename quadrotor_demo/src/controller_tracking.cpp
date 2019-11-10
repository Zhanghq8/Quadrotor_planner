#include "../include/quadrotor_demo/controller_tracking.h"

Controller_tracking::Controller_tracking(ros::NodeHandle* nodehandle):nh_(*nodehandle) { 
// constructor
    // ROS_INFO("In class constructor of Controller_tracking");
    setdronenum();
    setwaypoint_cnt();
    setvelocity();
    setpidgains();
    initVec();
    initSub();
    initPub();
    setpath();
    setgoalpos();
    
}

void Controller_tracking::initVec() {
    errorvector = vector<vector<double>>(drone_num, vector<double> (6, 0));
    posvector = vector<vector<double>> (drone_num, vector<double> (7, 0));
    goalposvector = vector<vector<double>> (drone_num, vector<double> (7, 0));

    diffvector = vector<vector<double>> (drone_num, vector<double> (3, 0));
    xyvelocity = vector<vector<double>> (drone_num, vector<double> (3, 0));
}

void Controller_tracking::setgoalpos()
{   
    for (int i=0; i<drone_num; i++) {
        goalposvector[i][0] = path[i][0].x;
        goalposvector[i][1] = path[i][0].y;
        // cout << "Setting goal point as " << i << " (" << goalposvector[i][0] << "," << goalposvector[i][1] << ")." << endl;
    }

}

void Controller_tracking::setwaypoint_cnt()
{   
    waypoint_cnt = vector<int> (drone_num, 1);
}

void Controller_tracking::setpath() {
    path = vector<vector<Vec2i>> { { {1,0}, {2,0}, {3,0} }, 
            { {10,-9}, {10,-8}, {10,-7} },
            { {-10,-9}, {-10,-8}, {-10,-7} } };
}

void Controller_tracking::setpidgains(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
}

void Controller_tracking::setvelocity(double x) {
    v = x;
}

void Controller_tracking::setdronenum(int num) {
    drone_num = num;
}

void Controller_tracking::initSub() {
    // ROS_INFO("Initializing Subscribers");  
    stop_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &Controller_tracking::eventCallback,this);
    current1pos_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &Controller_tracking::currentpos1Callback,this);
    current2pos_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &Controller_tracking::currentpos2Callback,this);
    current3pos_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &Controller_tracking::currentpos3Callback,this);
    // goalpos_sub_ = nh_.subscribe("/pose_pub", 1, &Controller_tracking::goalCallback,this); 
}

//member helper function to set up publishers;
void Controller_tracking::initPub() {
    // ROS_INFO("Initializing Publishers");
    control1input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
    control2input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone2/cmd_vel", 1, true);
    control3input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone3/cmd_vel", 1, true);
    ready_flag_pub_ = nh_.advertise<std_msgs::Bool>("/ready_flag", 1, true);
}

// void Controller_tracking::goalCallback(const geometry_msgs::PoseArray& goal) {
//     start_flag = true;
//     for (int i=0; i<drone_num; i++) {
//         for (int j=0; j<2; j++) {
//             if (j == 0) {
//                 goalposvector[i][j] = goal.poses[i].position.x;
//             }
//             if (j == 1) {
//                 goalposvector[i][j] = goal.poses[i].position.y;
//             } 
//         }
//     }
// }

void Controller_tracking::currentpos3Callback(const geometry_msgs::PoseStamped& odom3) {
    // cout << "aaaaaaaaaaaaa" << endl;
    posvector[2][0] = odom3.pose.position.x;
    posvector[2][1] = odom3.pose.position.y;
    posvector[2][2] = odom3.pose.position.z;
    posvector[2][3] = odom3.pose.orientation.x; 
    posvector[2][4] = odom3.pose.orientation.y;
    posvector[2][5] = odom3.pose.orientation.z;
    posvector[2][6] = odom3.pose.orientation.w;
}

void Controller_tracking::currentpos2Callback(const geometry_msgs::PoseStamped& odom2) {
    // cout << "ccccccccccccccccccc" << endl;
    posvector[1][0] = odom2.pose.position.x;
    posvector[1][1] = odom2.pose.position.y;
    posvector[1][2] = odom2.pose.position.z;
    posvector[1][3] = odom2.pose.orientation.x; 
    posvector[1][4] = odom2.pose.orientation.y;
    posvector[1][5] = odom2.pose.orientation.z;
    posvector[1][6] = odom2.pose.orientation.w;
}

void Controller_tracking::currentpos1Callback(const geometry_msgs::PoseStamped& odom1) {   
    posvector[0][0] = odom1.pose.position.x;
    posvector[0][1] = odom1.pose.position.y;
    posvector[0][2] = odom1.pose.position.z;
    posvector[0][3] = odom1.pose.orientation.x; 
    posvector[0][4] = odom1.pose.orientation.y;
    posvector[0][5] = odom1.pose.orientation.z;
    posvector[0][6] = odom1.pose.orientation.w;

    vector<double> pose_error(drone_num, 0);
    // if (start_flag == true && ready_flag == true) {
    for (int i=0; i<drone_num; i++) {
        for (int j=0; j<diffvector[i].size(); j++) {
            // cout << i << ", " << j << " " << goalposvector[i][j] << " " << posvector[i][j] << endl;
            if (j == int(diffvector[i].size()) - 1) {
                diffvector[i][j] = atan2(diffvector[i][j-1], diffvector[i][j-2]);
                pose_error[0] += sqrt(diffvector[i][j-1]*diffvector[i][j-1] + diffvector[i][j-2]*diffvector[i][j-2]);
            }
            else {
                diffvector[i][j] = goalposvector[i][j] - posvector[i][j];
            }
        }
    }
    
    if ((ready_flag_1 && ready_flag_2 && ready_flag_3) || (goalposvector[0][0] == 0 && goalposvector[2][0] == 0 && goalposvector[2][1] == 0)) {
        ready_flag = true;
    }
    else {
        ready_flag = false;
    }

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
            if (goalposvector[i][0] == path[i][path[i].size()-1].x && goalposvector[i][1] == path[i][path[i].size()-1].y) {
                xyvelocity[i][0] = kp*errorvector[i][1] + ki*errorvector[i][2] + kd*errorvector[i][3];
            }
            else {
                xyvelocity[i][0] = v;
            }
            // xyvelocity[i][0] = kp*errorvector[i][1] + ki*errorvector[i][2] + kd*errorvector[i][3];
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
        // cout << "drone "  << ": vx, " << xyvelocity[0][1] << endl;
        // cout << "drone "  << ": vy, " << xyvelocity[0][2] << endl;;
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
};
void Controller_tracking::eventCallback(const geometry_msgs::PoseStamped& odom1) 
{   
    for (int i=0; i<drone_num; i++) {

        int pathsize = path[i].size();
        // cout << "Xc pose: " << currentpos.x << " Xg pose: " << goalpos.x << " Yc pose: " << posvector[i][1] << " Yg pose: "<< goalpos.y <<endl;
        if ((abs(posvector[i][0] - goalposvector[i][0])<d*0.1) && (abs(posvector[i][1] - goalposvector[i][1])<d*0.1 ) 
            && (posvector[i][0] * goalposvector[i][0] > 0.01))
        {   
            if (i == 0) {
                if (waypoint_cnt[i] == pathsize) {
                    control1input.linear.x = 0.0;
                    control1input.linear.y = 0.0;
                    control1input.linear.z = 0.0;
                    control1input_pub_.publish(control1input);
                    ready_flag_1 = true;
                    // ROS_INFO("Finished...");
                    // ros::shutdown();
                }
                else {   
                    // cout << "Waypoint " << waypoint_cnt[i] << " reached. ";
                    // cout << "currentpos: " << posvector[i][0] << " " << posvector[i][1] << endl;
                    waypoint_cnt[i]++;
                    goalposvector[i][0] = path[i][waypoint_cnt[i]-1].x;
                    goalposvector[i][1] = path[i][waypoint_cnt[i]-1].y;
                }
            }
            else if (i == 1) {
                if (waypoint_cnt[i] == pathsize) {
                    control2input.linear.x = 0.0;
                    control2input.linear.y = 0.0;
                    control2input.linear.z = 0.0;
                    control2input_pub_.publish(control2input);
                    ready_flag_2 = true;
                    // ROS_INFO("Finished...");
                    // ros::shutdown();
                }
                else {   
                    // cout << "Waypoint " << waypoint_cnt[i] << " reached. ";
                    // cout << "currentpos: " << posvector[i][0] << " " << posvector[i][1] << endl;
                    waypoint_cnt[i]++;
                    goalposvector[i][0] = path[i][waypoint_cnt[i]-1].x;
                    goalposvector[i][1] = path[i][waypoint_cnt[i]-1].y;
                }
            }
            else {
                if (waypoint_cnt[i] == pathsize) {
                        control3input.linear.x = 0.0;
                        control3input.linear.y = 0.0;
                        control3input.linear.z = 0.0;
                        control3input_pub_.publish(control3input);
                        ready_flag_3 = true;
                        // ROS_INFO("Finished...");
                        // ros::shutdown();
                }
                else {   
                    // cout << "Waypoint " << waypoint_cnt[i] << " reached. ";
                    // cout << "currentpos: " << posvector[i][0] << " " << posvector[i][1] << endl;
                    waypoint_cnt[i]++;
                    goalposvector[i][0] = path[i][waypoint_cnt[i]-1].x;
                    goalposvector[i][1] = path[i][waypoint_cnt[i]-1].y;
                }
            }

        }
    }
};


int main(int argc, char** argv)
{

    ros::init(argc, argv, "tracking"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Controller_tracking controller_tracking(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing controller...");
    ros::spin();
    return 0;
}