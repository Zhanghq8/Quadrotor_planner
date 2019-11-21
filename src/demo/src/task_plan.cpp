#include "../include/quadrotor_demo/task_plan.h"

Task_plan::Task_plan(ros::NodeHandle* nodehandle):nh_(*nodehandle) { 
// constructor
    // ROS_INFO("In class constructor of Task_plan");
    setdronenum();
    initVec();
    initSub();
    initPub();
    // generateip();
    // plan();
}

void Task_plan::initSub() {
    // ROS_INFO("Initializing Subscribers");  

    pos1_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &Task_plan::pos1Callback,this);
    pos2_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &Task_plan::pos2Callback,this);
    pos3_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &Task_plan::pos3Callback,this);
    readyflag_sub_ = nh_.subscribe("/ready_flag", 1, &Task_plan::flagCallback,this); 
}

//member helper function to set up publishers;
void Task_plan::initPub() {
    // ROS_INFO("Initializing Publishers");
    task_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/pose_pub", 1, true); 
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markerarray", 10);
}

void Task_plan::setdronenum(int num) {
    drone_num = num;
}

void Task_plan::generateip() {
    double diff12 = 0;
    double diff23 = 0;
    double diff13 = 0;
    vector<double> diff (drone_num, 0);
    double min_diff = *min_element(diff.begin(), diff.end());
    while (min_diff <= 0.6) {
        for (int i=0; i<drone_num; i++) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> x(-10.0, 10.0);
            std::uniform_real_distribution<> y(-10.0, 10.0);
            interestedpoints[i][0] = x(gen);
            interestedpoints[i][1] = y(gen);
        }
        for (int i=0; i<interestedpoints.size(); i++) {
            if (i < interestedpoints.size()-1) {
                diff[i] = distance(interestedpoints[i][0], interestedpoints[i][1],
                 interestedpoints[i+1][0], interestedpoints[i+1][1]);
            }
            else {
                diff[i] = distance(interestedpoints[0][0], interestedpoints[0][1], 
                    interestedpoints[i][0], interestedpoints[i][1]);
            }
            
        }
        min_diff = *min_element(diff.begin(), diff.end());
        // diff12 = distance(interestedpoints[0][0], interestedpoints[0][1], interestedpoints[1][0], interestedpoints[1][1]);
        // diff23 = distance(interestedpoints[1][0], interestedpoints[1][1], interestedpoints[2][0], interestedpoints[2][1]);
        // diff13 = distance(interestedpoints[0][0], interestedpoints[0][1], interestedpoints[2][0], interestedpoints[2][1]);
    } 


    visualization_msgs::MarkerArray cube_marker_array;

    visualization_msgs::Marker cube_marker;
    cube_marker.type = visualization_msgs::Marker::CUBE_LIST;
    cube_marker.action = visualization_msgs::Marker::ADD;
    cube_marker.ns = "cubes";
    cube_marker.scale.x = 1;
    cube_marker.scale.y = 1;
    cube_marker.scale.z = 1;
    cube_marker.header.frame_id = "/world"; // shoudl become wepod_base or the like
    cube_marker.color.a = 1.0; // Don't forget to set the alpha!
    cube_marker.color.r = 1.0;
    cube_marker.id = 0;
    cube_marker.lifetime = ros::Duration();
    // ros::Duration lifetime;
    // arrow_marker.lifetime = lifetime.fromSec(0.04); // lifetime of 40ms : 25Hz


    for (int i=0; i<interestedpoints.size(); i++) {
        geometry_msgs::Point obj;
        obj.x = interestedpoints[i][0];
        obj.y = interestedpoints[i][1];
        obj.z = 0;
        cube_marker.points.push_back(obj);
        cube_marker.colors.push_back(cube_marker.color);
        cube_marker.header.stamp = ros::Time::now();
        cube_marker_array.markers.push_back(cube_marker);
        // Adding a text object here
    }

    marker_pub_.publish(cube_marker_array);
}

void Task_plan::initVec() {
    interestedpoints = vector<vector<double>>(drone_num, vector<double> (7, 0));
    posvector = vector<vector<double>> (drone_num, vector<double> (7, 0));
    // this vector store the target pos index for drones in order
    // task_index = vector<int> (drone_num, 0);
}


void Task_plan::pos1Callback(const geometry_msgs::PoseStamped& odom1) {
    posvector[0][0] = odom1.pose.position.x;
    posvector[0][1] = odom1.pose.position.y;
    posvector[0][2] = odom1.pose.position.z;
    posvector[0][3] = odom1.pose.orientation.x; 
    posvector[0][4] = odom1.pose.orientation.y;
    posvector[0][5] = odom1.pose.orientation.z;
    posvector[0][6] = odom1.pose.orientation.w;
}

void Task_plan::pos2Callback(const geometry_msgs::PoseStamped& odom2) {
    posvector[1][0] = odom2.pose.position.x;
    posvector[1][1] = odom2.pose.position.y;
    posvector[1][2] = odom2.pose.position.z;
    posvector[1][3] = odom2.pose.orientation.x; 
    posvector[1][4] = odom2.pose.orientation.y;
    posvector[1][5] = odom2.pose.orientation.z;
    posvector[1][6] = odom2.pose.orientation.w;
}

void Task_plan::pos3Callback(const geometry_msgs::PoseStamped& odom3) {   

    posvector[2][0] = odom3.pose.position.x;
    posvector[2][1] = odom3.pose.position.y;
    posvector[2][2] = odom3.pose.position.z;
    posvector[2][3] = odom3.pose.orientation.x; 
    posvector[2][4] = odom3.pose.orientation.y;
    posvector[2][5] = odom3.pose.orientation.z;
    posvector[2][6] = odom3.pose.orientation.w;
}

// Hidden input: interestedip, pos1, pos2, pos3
// output: geometry_msgs::PoseArray type publish
void Task_plan::plan() {
    //TODO: how to plan
    vector<vector<double>> costMatrix;
    costMatrix = vector<vector<double>>(drone_num, vector<double> (3, 0));

    for (int i=0; i<costMatrix.size(); i++) {
        for (int j=0; j<costMatrix[i].size(); j++) {
            costMatrix[i][j] = distance(posvector[i][0], posvector[i][1], interestedpoints[j][0], interestedpoints[j][1]);
        }
    }

    cout << "Interested points: " << endl;;
    for (int i=0; i<interestedpoints.size(); i++) {
        cout << "point " << i << ": " << "( " << interestedpoints[i][0] << "," << interestedpoints[i][1] << " )" << endl; 
    }

    HungarianAlgorithm HungAlgo;
    double cost = HungAlgo.Solve(costMatrix, task_index);
    cout << "Task assignment: " << endl;
    for (int i=0; i<task_index.size(); i++) {
        cout << "drone "<< i+1 << ": " << "task " << task_index[i] << endl; 
    }

    geometry_msgs::PoseArray task_pose;
    for (int i=0; i<drone_num; i++) {
        geometry_msgs::Pose pose;
        pose.position.x = interestedpoints[task_index[i]][0];
        pose.position.y = interestedpoints[task_index[i]][1];
        task_pose.poses.push_back(pose);
    }
    task_pub_.publish(task_pose);
}

void Task_plan::flagCallback(const std_msgs::Bool& flag_msg) {
    flag = flag_msg.data;
    if (flag == true) {
        generateip();
        plan();
    }
}

double Task_plan::distance(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "Task_plan"); //node name

    ros::NodeHandle nh; 
    Task_plan task_plan(&nh);  

    ROS_INFO("Initializing task planning...");
    ros::Rate loop_rate(0.5); // 5Hz
    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }
    // ros::spin();
    return 0;
}

