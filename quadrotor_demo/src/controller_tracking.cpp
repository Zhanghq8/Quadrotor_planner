#include "../include/quadrotor_demo/controller_tracking.h"

Controller_tracking::Controller_tracking(ros::NodeHandle* nodehandle):nh_(*nodehandle) { 
// constructor
    // ROS_INFO("In class constructor of Controller_tracking");
    setdronenum();
    setvelocity();
    setpidgains();
    initVec();
    initSub();
    initPub();
  
}

void Controller_tracking::initVec() {
    errorvector = vector<vector<double>>(drone_num, vector<double> (6, 0));
    posvector = vector<vector<double>> (drone_num, vector<double> (7, 0));
    goalposvector = vector<vector<double>> (drone_num, vector<double> (7, 0));
    goalposvector[0] = {0.5,0.5,0,0,0,0,1};
    goalposvector[1] = {0.5,14.5,0,0,0,0,1};
    goalposvector[2] = {14.5,14.5,0,0,0,0,1};
    waypoint_cnt = vector<int> (drone_num, 0);
    diffvector = vector<vector<double>> (drone_num, vector<double> (3, 0));
    xyvelocity = vector<vector<double>> (drone_num, vector<double> (3, 0));
    switchFlag01 = vector<bool> (3, false);
    switchFlag12 = vector<bool> (3, false);
    switchFlag23 = vector<bool> (3, false);
    pidFlag = vector<bool> (3, false);
    updateMapFlag = vector<bool> (3, false);
    updateComplete = vector<bool> (3, false);
    // path = vector<vector<vector<Vec2i>>> (drone_num);
}

void Controller_tracking::setgoalpos() {   
    for (int i=0; i<drone_num; i++) {
        if (!sensorPath[i].empty() && !sensorPath[i][0].empty()) {
            goalposvector[i][0] = sensorPath[i][0][0].x;
            goalposvector[i][1] = sensorPath[i][0][0].y;
        } else {
            goalposvector[i][0] = posvector[i][0];
            goalposvector[i][1] = posvector[i][1];
        }
        std::cout << "Setting goal point for Drone(sensor) " << i << " as (" << goalposvector[i][0] << "," << goalposvector[i][1] << ")." << std::endl;
    }
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
    event1_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &Controller_tracking::event1Callback,this);
    event2_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &Controller_tracking::event2Callback,this);
    event3_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &Controller_tracking::event3Callback,this);
    update1_complete_sub_ = nh_.subscribe("/update1_complete", 1, &Controller_tracking::update1CompleteCallback,this);
    update2_complete_sub_ = nh_.subscribe("/update2_complete", 1, &Controller_tracking::update2CompleteCallback,this);
    update3_complete_sub_ = nh_.subscribe("/update3_complete", 1, &Controller_tracking::update3CompleteCallback,this);
    current1pos_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &Controller_tracking::currentpos1Callback,this);
    current2pos_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &Controller_tracking::currentpos2Callback,this);
    current3pos_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &Controller_tracking::currentpos3Callback,this);
    path_sub_ = nh_.subscribe("/sensor_path", 1, &Controller_tracking::pathCallback,this); 
}

//member helper function to set up publishers;
void Controller_tracking::initPub() {
    // ROS_INFO("Initializing Publishers");
    update1Map_flag_pub_ = nh_.advertise<std_msgs::Bool>("/updatemap1", 1, true);
    update2Map_flag_pub_ = nh_.advertise<std_msgs::Bool>("/updatemap2", 1, true);
    update3Map_flag_pub_ = nh_.advertise<std_msgs::Bool>("/updatemap3", 1, true);
    control1input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
    control2input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone2/cmd_vel", 1, true);
    control3input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone3/cmd_vel", 1, true);
    updategraph_flag_pub_ = nh_.advertise<std_msgs::Bool>("/updategraph_flag", 1, true);
}

void Controller_tracking::pathCallback(const quadrotor_demo::final_path& path) {
    sensorPath = {};
    for(std::vector<quadrotor_demo::pathes>::const_iterator itr = path.final_path.begin(); itr != path.final_path.end(); ++itr) {   
        bool isEmpty = itr->empty;
        string name = itr->path_name;
        vector<vector<Vec2i>> singleSensorPath;
        if (isEmpty == true) {
            sensorPath.emplace_back(singleSensorPath);
        } else {
            for (std::vector<quadrotor_demo::path>::const_iterator pdata = itr->pathes_data.begin(); pdata != itr->pathes_data.end(); ++pdata) {
                vector<Vec2i> pose;
                for (std::vector<quadrotor_demo::pose>::const_iterator posdata = pdata->path.begin(); posdata != pdata->path.end(); ++posdata) {
                    // std::cout << posdata->x << " " << posdata->y << " " << posdata->z << std::endl;
                    Vec2i coordinate;
                    coordinate.id = posdata->id;
                    coordinate.x = posdata->x;
                    coordinate.y = posdata->y;
                    coordinate.z = posdata->z;
                    coordinate.xcoord = posdata->xcoordinate;
                    coordinate.ycoord = posdata->ycoordinate;
                    pose.emplace_back(coordinate);
                }
                singleSensorPath.emplace_back(pose);
            }
            sensorPath.emplace_back(singleSensorPath);
        }
    }
    // std::cout << sensorPath[0].size() << " " << sensorPath[1].size() << sensorPath[2].size() << std::endl; 
    start_flag = true;
    updateMapFlag = vector<bool> (3, false);
    updateComplete = vector<bool> (3, false);
    //============================== DEBUG ========================//
    //=============================================================//
    // for (int i=0; i<sensorPath.size(); i++) {
    //     std::cout << "================================" << std::endl;
    //     for (int j=0; j<sensorPath[i].size(); j++) {
    //         for (int k=0; k<sensorPath[i][j].size(); k++) {
    //             std::cout << sensorPath[i][j][k].id << " " << sensorPath[i][j][k].x << " " << sensorPath[i][j][k].y << std::endl;
    //         }
    //         std::cout << "sensorPath " << i << " " << j << " size: " << sensorPath[i][j].size() << std::endl;
    //     }
    //     std::cout << "sensorPath " << i << " size: " << sensorPath[i].size() << std::endl;
    // }
    // std::cout << "sensorPath size:" << sensorPath.size() << std::endl;
    //=============================================================//
    //=============================================================//
    setgoalpos();
    waypoint_cnt = vector<int> (drone_num, 0);
}

void Controller_tracking::currentpos3Callback(const geometry_msgs::PoseStamped& odom3) {
    posvector[2][0] = odom3.pose.position.x;
    posvector[2][1] = odom3.pose.position.y;
    posvector[2][2] = odom3.pose.position.z;
    posvector[2][3] = odom3.pose.orientation.x; 
    posvector[2][4] = odom3.pose.orientation.y;
    posvector[2][5] = odom3.pose.orientation.z;
    posvector[2][6] = odom3.pose.orientation.w;
}

void Controller_tracking::currentpos2Callback(const geometry_msgs::PoseStamped& odom2) {
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

    // set a flag in case this run before we get a path

    vector<double> pose_error(drone_num, 0);
    // if (start_flag == true && ready_flag == true) {
    for (int i=0; i<drone_num; i++) {
        for (int j=0; j<diffvector[i].size(); j++) {
            // cout << i << ", " << j << " " << goalposvector[i][j] << " " << posvector[i][j] << endl;
            if (j == int(diffvector[i].size()) - 1) {
                diffvector[i][j] = atan2(diffvector[i][j-1], diffvector[i][j-2]);
                pose_error[0] += sqrt(diffvector[i][j-1]*diffvector[i][j-1] + diffvector[i][j-2]*diffvector[i][j-2]);
            } else {
                diffvector[i][j] = goalposvector[i][j] - posvector[i][j];
            }
        }
    }

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
        // if (goalposvector[i][0] == path[i][path[i].size()-1].x && goalposvector[i][1] == path[i][path[i].size()-1].y) {
        //     xyvelocity[i][0] = kp*errorvector[i][1] + ki*errorvector[i][2] + kd*errorvector[i][3];
        // }
        // else {
        //     xyvelocity[i][0] = v;
        // }

        // if (pidFlag[i]) {
        //     xyvelocity[i][0] = kp*errorvector[i][1] + ki*errorvector[i][2] + kd*errorvector[i][3];
        // } else {
        //     xyvelocity[i][0] = v;
        // }
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

void Controller_tracking::event1Callback(const geometry_msgs::PoseStamped& odom1) {
    if (!switchFlag0 && !switchFlag1 && !switchFlag2 && start_flag && !(updateComplete[0] && updateComplete[1] && updateComplete[2])) {
        // std::cout << "event1: " << switchFlag0 << " " << switchFlag1 << " " << switchFlag2 << " " << start_flag << std::endl;
        // std::cout << "flag: " << updateComplete[0] << " " << updateComplete[1] << " " << updateComplete[2] << std::endl;
        for (int i=0; i<drone_num; i++) {
            // check if the there is task for dronei
            if (!sensorPath[i].empty()) {
                if (!sensorPath[i][0].empty()) {
                    // std::cout << "currentpos: " << posvector[i][0] << " " << posvector[i][1] << std::endl;
                    // std::cout << "goalpos: " << goalposvector[i][0] << " " << goalposvector[i][1] << std::endl;
                    // std::cout << "waypoint: " << waypoint_cnt[i] << std::endl;
                    if ((abs(posvector[i][0] - goalposvector[i][0])<d*0.1) && (abs(posvector[i][1] - goalposvector[i][1])<d*0.1) 
                        && (posvector[i][0] * goalposvector[i][0] > 0.01)) {
                        if (waypoint_cnt[i] == sensorPath[i][0].size() - 1) {
                            std::cout << "Event1 for Drone(sensor) " << i << " Waypoint " << waypoint_cnt[i] << " reached. " << std::endl;
                            std::cout << "Event1 for Drone(sensor) " << i << " reached!!!!!!. " << std::endl;
                            updateMapFlag[i] = true;
                            control1input.linear.x = 0.0;
                            control1input.linear.y = 0.0;
                            control1input.linear.z = 0.0;
                            control1input_pub_.publish(control1input);
                            switchFlag01[i] = true;
                        }
                        else {   
                            std::cout << "Event1 for Drone(sensor) " << i << " Waypoint " << waypoint_cnt[i] << " reached. " << std::endl;
                            // if (waypoint_cnt[i] == sensorPath[i][1].size() - 2) {
                            //     pidFlag[i] = true;
                            // } else {
                            //     pidFlag[i] = false;
                            // }
                            waypoint_cnt[i]++;
                            goalposvector[i][0] = sensorPath[i][0][waypoint_cnt[i]].x;
                            goalposvector[i][1] = sensorPath[i][0][waypoint_cnt[i]].y;
                        }
                    } 
                } else {
                    switchFlag01[i] = true;
                    updateComplete[i] = true;
                }
            } else {
                switchFlag01[i] = true;
                updateComplete[i] = true;
            }
        }
        switchFlag0 = switchFlag01[0] && switchFlag01[1] && switchFlag01[2];
        if (switchFlag0) {
            waypoint_cnt = vector<int> (drone_num, 0);
            updateMap();
        }
    }
    
}

void Controller_tracking::event2Callback(const geometry_msgs::PoseStamped& odom2) {
    if (switchFlag0 && !switchFlag1 && !switchFlag2 && start_flag && (updateComplete[0] && updateComplete[1] && updateComplete[2])) {
        for (int i=0; i<drone_num; i++) {
        // check if the there is task for dronei
            if (!sensorPath[i].empty()) {

                if (!sensorPath[i][1].empty()) {
                    if ((abs(posvector[i][0] - goalposvector[i][0])<d*0.1) && (abs(posvector[i][1] - goalposvector[i][1])<d*0.1 ) 
                        && (posvector[i][0] * goalposvector[i][0] > 0.01)) {
                        if (waypoint_cnt[i] == sensorPath[i][1].size() - 1) {
                            std::cout << "Event2 for Drone(sensor) " << i << " Waypoint " << waypoint_cnt[i] << " reached. " << std::endl;
                            std::cout << "Event2 for Drone(sensor) " << i << " reached!!!!!!. " << std::endl;
                            updateMapFlag[i] = true;
                            control1input.linear.x = 0.0;
                            control1input.linear.y = 0.0;
                            control1input.linear.z = 0.0;
                            control1input_pub_.publish(control1input);
                            switchFlag12[i] = true;
                        }
                        else {   
                            std::cout << "Event2 for Drone(sensor) " << i << " Waypoint " << waypoint_cnt[i] << " reached. " << std::endl;
                            // if (waypoint_cnt[i] == sensorPath[i][1].size() - 2) {
                            //     pidFlag[i] = true;
                            // } else {
                            //     pidFlag[i] = false;
                            // }
                            waypoint_cnt[i]++;
                            goalposvector[i][0] = sensorPath[i][1][waypoint_cnt[i]].x;
                            goalposvector[i][1] = sensorPath[i][1][waypoint_cnt[i]].y;
                        }
                    }
                } else {
                    switchFlag12[i] = true;
                    updateComplete[i] = true;
                }
            } else {
                switchFlag12[i] = true;
                updateComplete[i] = true;
            }
        }
        switchFlag1 = switchFlag12[0] && switchFlag12[1] && switchFlag12[2];
        if (switchFlag1) {
            waypoint_cnt = vector<int> (drone_num, 0);
            updateMap();
        }
    }    
}

void Controller_tracking::event3Callback(const geometry_msgs::PoseStamped& odom3) {
    if (switchFlag0 && switchFlag1 && !switchFlag2 && start_flag && (updateComplete[0] && updateComplete[1] && updateComplete[2])) {
        for (int i=0; i<drone_num; i++) {
        // check if the there is task for dronei
            if (!sensorPath[i].empty()) {

                if (!sensorPath[i][2].empty()) {
                    if ((abs(posvector[i][0] - goalposvector[i][0])<d*0.1) && (abs(posvector[i][1] - goalposvector[i][1])<d*0.1 ) 
                        && (posvector[i][0] * goalposvector[i][0] > 0.01)) {
                        if (waypoint_cnt[i] == sensorPath[i][2].size() - 1) {
                            std::cout << "Event3 for Drone(sensor) " << i << " Waypoint " << waypoint_cnt[i] << " reached. " << std::endl;
                            std::cout << "Event3 for Drone(sensor) " << i << " reached!!!!!!. " << std::endl;
                            updateMapFlag[i] = true;
                            control1input.linear.x = 0.0;
                            control1input.linear.y = 0.0;
                            control1input.linear.z = 0.0;
                            control1input_pub_.publish(control1input);
                            switchFlag23[i] = true;
                        } else {   
                            std::cout << "Event3 for Drone(sensor) " << i << " Waypoint " << waypoint_cnt[i] << " reached. " << std::endl;
                            // cout << "currentpos: " << posvector[i][0] << " " << posvector[i][1] << endl;
                            // if (waypoint_cnt[i] == sensorPath[i][1].size() - 2) {
                            //     pidFlag[i] = true;
                            // } else {
                            //     pidFlag[i] = false;
                            // }
                            waypoint_cnt[i]++;
                            goalposvector[i][0] = sensorPath[i][2][waypoint_cnt[i]].x;
                            goalposvector[i][1] = sensorPath[i][2][waypoint_cnt[i]].y;
                        }
                    }
                } else {
                    switchFlag23[i] = true;
                }
            } else {
                switchFlag23[i] = true;
            }
            switchFlag2 = switchFlag23[0] && switchFlag23[1] && switchFlag23[2];

            if (switchFlag2) {
                updateMap();
                ros::Rate loop_rate(5);
                loop_rate.sleep();
                if (updateComplete[0] && updateComplete[1] && updateComplete[2] && switchFlag2) {
                    std_msgs::Bool flag;
                    flag.data = switchFlag2;
                    updategraph_flag_pub_.publish(flag);
                }
                start_flag = false;
                switchFlag0 = false;
                switchFlag1 = false;
                switchFlag2 = false;
                switchFlag01 = vector<bool> (3, false);
                switchFlag12 = vector<bool> (3, false);
                switchFlag23 = vector<bool> (3, false);
                updateMapFlag = vector<bool> (3, false);
                updateComplete = vector<bool> (3, false);
            }
        }
    }
}

void Controller_tracking::updateMap() {
    for (int i=0; i<updateMapFlag.size(); i++) {
        std_msgs::Bool updateMap;
        updateMap.data = updateMapFlag[i];
        if (i == 0) {
            update1Map_flag_pub_.publish(updateMap);
        } else if (i == 1) {
            update2Map_flag_pub_.publish(updateMap);
        } else {
            update3Map_flag_pub_.publish(updateMap);
        }
    }
}

void Controller_tracking::update1CompleteCallback(const std_msgs::Bool& flag1) {
    updateComplete[0] = flag1.data;
}

void Controller_tracking::update2CompleteCallback(const std_msgs::Bool& flag2) {
    updateComplete[1] = flag2.data;
}

void Controller_tracking::update3CompleteCallback(const std_msgs::Bool& flag3) {
    updateComplete[2] = flag3.data;
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