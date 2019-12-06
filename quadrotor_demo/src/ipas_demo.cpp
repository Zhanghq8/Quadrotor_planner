#include "../include/quadrotor_demo/ipas_demo.h"

IpasDemo::IpasDemo(ros::NodeHandle* nodehandle, std::vector<Task>& tasks_data, std::vector<AutoVehicle>& agent, 
		Eigen::MatrixXi& comm, int64_t num_vehicle, int64_t num_tasks, int64_t num_sensors, int64_t num_row, 
			int64_t num_col, std::vector<std::vector<int64_t>> range_idx) 
			: nh_(*nodehandle), tasks_data_(tasks_data), agents_(agent), comm_(comm), num_vehicle_(num_vehicle), 
			num_tasks_(num_tasks), num_sensors_(num_sensors), num_row_(num_row), num_col_(num_col),
			range_idx_(range_idx) { 
	// constructor
 	// ROS_INFO("In class constructor of IpasDemo");
	assert(num_sensors_ == 3 && "The number of sensors in this demo must be strictly equal to 3!");
	init();
    initSub();
    initPub();
}

IpasDemo::~IpasDemo() {

}

void IpasDemo::initMap() {
    uncertain_grid = GridGraph::CreateSquareGrid(num_row_,num_col_,1,vehicle_team_,tasks_);
	uncertain_graph = GridGraph::BuildGraphFromSquareGrid(uncertain_grid, true);

    // Build true map
    true_grid = GridGraph::CreateSquareGrid(num_row_,num_col_,1);
    for(auto rg: range_idx_){
        for(int ii = rg[0]; ii<rg[1];ii++){
            true_grid->SetObstacleRegionLabel(ii,1);
        }
    }
    true_grid->SetObstacleRegionLabel(4,1);
    true_grid->SetObstacleRegionLabel(5,1);
    true_grid->SetObstacleRegionLabel(6,1);
    true_grid->SetObstacleRegionLabel(9,1);
    true_grid->SetObstacleRegionLabel(14,1);
    true_grid->SetObstacleRegionLabel(20,1);
    true_grid->SetObstacleRegionLabel(21,1);
    true_grid->SetObstacleRegionLabel(22,1);
    true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);
    //===============================================================================================//
    //============================================= CBBA ============================================//
    //===============================================================================================//   
    // Initialize local_grid and local_graph
    IPASMeasurement::InitLocalGraph(vehicle_team_,uncertain_grid);  

    // //======================================== TEST ===============================================//
    // std::vector<Vertex_t<SquareCell*>*> vts = true_graph->GetAllVertex();
    // for(auto vt: vts){
    //     std::cout << "Vertex " << vt->state_->id_ << ": "<< std::endl;
    //     std::cout << "The (x, y) : (" << vt->state_->position_.x << ", " << vt->state_->position_.y << ")" <<std::endl;
    //     std::cout << "The (row,col) : (" << vt->state_->coordinate_.x << ", "<< vt->state_->coordinate_.y << ")" <<std::endl; 
    //     std::cout << "The probability p is " << vt->state_->p_ << ", and IG is " << vt->state_->ig_ <<std::endl;
    //     std::cout << "The neighbors are: " <<std::endl;
    //     std::vector<Vertex_t<SquareCell*>*> neighbs = vt->GetNeighbours();
    //     for(auto nb: neighbs){
    //         auto ecost = vt->GetEdgeCost(nb);
    //         std::cout << "Vertex " << nb->state_->id_ << ". The edge cost is: " << ecost <<std::endl;
    //     }
    //     std::cout << "=============================================" <<std::endl;
    // }   

}


void IpasDemo::init() {
	updatemap_flag = true;
	ipas_tt = 0;
	tasks_ = TasksSet(tasks_data_);
	vehicle_team_ = IPASMeasurement::ConstructAutoTeam(agents_);
	initMap();
}


void IpasDemo::initSub() {
    // ROS_INFO("Initializing Subscribers");  
    currentpos1_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &IpasDemo::currentpos1Callback,this);
    currentpos2_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &IpasDemo::currentpos2Callback,this);
    currentpos3_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &IpasDemo::currentpos3Callback,this);
    updatemapflag_sub_ = nh_.subscribe("/updatemap_flag", 1, &IpasDemo::updatemapflagCallback,this);
    // goalpos_sub_ = nh_.subscribe("/pose_pub", 1, &IpasDemo::goalCallback,this); 
}

void IpasDemo::initPub() {
    // ROS_INFO("Initializing Publishers");
    task_pub_ = nh_.advertise<quadrotor_demo::final_path>("/sensor_path", 1, true); 
    // marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markerarray", 10);
    // control1input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
    // control2input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone2/cmd_vel", 1, true);
    // control3input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone3/cmd_vel", 1, true);
    // ready_flag_pub_ = nh_.advertise<std_msgs::Bool>("/ready_flag", 1, true);
}

void IpasDemo::currentpos1Callback(const geometry_msgs::PoseStamped& odom1) {
    // posvector[0][0] = odom1.pose.position.x;
    // posvector[0][1] = odom1.pose.position.y;
    // posvector[0][2] = odom1.pose.position.z;
    // posvector[0][3] = odom1.pose.orientation.x; 
    // posvector[0][4] = odom1.pose.orientation.y;
    // posvector[0][5] = odom1.pose.orientation.z;
    // posvector[0][6] = odom1.pose.orientation.w;
}

void IpasDemo::currentpos2Callback(const geometry_msgs::PoseStamped& odom2) {
    // posvector[1][0] = odom2.pose.position.x;
    // posvector[1][1] = odom2.pose.position.y;
    // posvector[1][2] = odom2.pose.position.z;
    // posvector[1][3] = odom2.pose.orientation.x; 
    // posvector[1][4] = odom2.pose.orientation.y;
    // posvector[1][5] = odom2.pose.orientation.z;
    // posvector[1][6] = odom2.pose.orientation.w;
}

void IpasDemo::currentpos3Callback(const geometry_msgs::PoseStamped& odom3) {   

    // posvector[2][0] = odom3.pose.position.x;
    // posvector[2][1] = odom3.pose.position.y;
    // posvector[2][2] = odom3.pose.position.z;
    // posvector[2][3] = odom3.pose.orientation.x; 
    // posvector[2][4] = odom3.pose.orientation.y;
    // posvector[2][5] = odom3.pose.orientation.z;
    // posvector[2][6] = odom3.pose.orientation.w;
}

void IpasDemo::updatemapflagCallback(const std_msgs::Bool& flag_msg) {
    updatemap_flag = flag_msg.data;
    if (updatemap_flag == true) {
    	mobilePath();
    }
}

void IpasDemo::mobilePath() {
	ipas_tt ++;
	std::cout << "Iteration: " << ipas_tt << std::endl;
    // Implement the CBBA to determine the task assignment
    CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
    // Compute the path for the auto team while satisfying its local assignment
    std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
    //=============================================================//
    //============================== DEBUG ========================//
    //=============================================================//
    for(auto p: path_ltl_){
    	if (p.first < 3) {
	        std::cout << "The path for vehicle " << p.first << " is: ";
	        for(auto v: p.second){
	            std::cout << v->id_ <<", ";
	        }
	        std::cout << std::endl;
    	}
    }
    //=============================================================//
    //=============================================================//
    //=============================================================//
    
    // Check whether the IPAS convergence is achieved
    bool flag_IPAS = IPASMeasurement::IPASConvergence(vehicle_team_,path_ltl_);
    if (flag_IPAS == true) {
    	std::cout << "The required iteration is " << ipas_tt <<std::endl; 
        ROS_INFO("Finished...");
        ros::shutdown();
    } else {
    	sensorPath();
    }
}

void IpasDemo::sensorPath() {
    //===============================================================================================//
    //============================================= IPAS ============================================//
    //===============================================================================================// 
    IPASMeasurement::ComputeHotSpots(vehicle_team_,tasks_);
    TasksSet sensing_tasks_ = IPASMeasurement::ConstructMeasurementTasks(vehicle_team_);
    hotspots.clear();
    hotspots = sensing_tasks_.GetHotspots();
    CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);

    std::map<int64_t,Path_t<SquareCell*>> path_sensing_ = IPASMeasurement::GeneratePaths(vehicle_team_,sensing_tasks_,TaskType::MEASURE);
    for(auto p: path_sensing_){
    	if (p.first >= 3) {
	        std::cout << "The path for sensor " << p.first << " is: ";
	        for(auto v: p.second){
	            std::cout << v->id_ <<", ";
                std::cout << "The (x, y) : (" << v->position_.x << ", " << v->position_.y << ")" <<", ";
        		std::cout << "The (row,col) : (" << v->coordinate_.x << ", "<< v->coordinate_.y << ")" <<". ";
	        }
	        std::cout << std::endl;
    	}
    }
    pathesPub(path_sensing_);
}

void IpasDemo::pathesPub(const std::map<int64_t,Path_t<SquareCell*>>& pathes) {
	quadrotor_demo::final_path finalpath_msg;
	// int path_size = pathes.size();
    for(auto p: pathes){
    	if (p.first >= 3) {
    		quadrotor_demo::pathes pathes_msg;
        	pathes_msg.path_name = std::string("Drone ") + std::string(std::to_string(p.first-2));
        	pathes_msg.empty = true;
        	quadrotor_demo::path path_msg;
	        for(auto v: p.second){
	        	pathes_msg.empty = false;
	        	quadrotor_demo::pose pose_msg;
	        	pose_msg.x = v->position_.x;
	        	pose_msg.y = v->position_.y;
	        	pose_msg.id = v->id_;
	        	pose_msg.xcoordinate = v->coordinate_.x;
	        	pose_msg.ycoordinate = v->coordinate_.y;
	        	path_msg.path.push_back(pose_msg);
	        	if (hotspots.count(v->id_) && hotspots.size() != 1) {
	        		hotspots.erase(v->id_);
	        		pathes_msg.pathes_data.push_back(path_msg);
	        		path_msg.path.clear();
	        		path_msg.path.push_back(pose_msg);
	        	}
	        }
	        pathes_msg.pathes_data.push_back(path_msg);
	        finalpath_msg.final_path.push_back(pathes_msg);
    	}
    }
    task_pub_.publish(finalpath_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ipasdemo"); //node name
    ros::NodeHandle nh; 


    //===============================================================================================//
    //===========================================Task - Agent========================================//
    //===============================================================================================//
    int64_t num_vehicle = 6;
    int64_t num_tasks = 6;
    // Tasks 
    // Define the task information:
    // Index of task, AP value(ltl), Position, Task Type, Number of vehicle
    std::vector<Task> tasks_data = {Task(0,2,{67},TaskType::RESCUE,num_vehicle),
                                	Task(1,3,{76},TaskType::RESCUE,num_vehicle),
                                	Task(2,4,{139},TaskType::RESCUE,num_vehicle),
                                	Task(3,5,{180},TaskType::RESCUE,num_vehicle),
                                	Task(4,6,{215},TaskType::RESCUE,num_vehicle),
                                	Task(5,7,{309},TaskType::RESCUE,num_vehicle)};
    // Auto Vehicle Team
    // Index of drone, Initial position, # of drones, Communicate network, Task Type, # of tasks
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,num_vehicle);
    int64_t num_sensors = 3;
    std::vector<AutoVehicle> agents = { AutoVehicle(0,0,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    	AutoVehicle(1,380,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    	AutoVehicle(2,399,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    	AutoVehicle(3,0,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    	AutoVehicle(4,380,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    	AutoVehicle(5,399,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors) };

    int64_t num_row = 20;
    int64_t num_col = 20;
    std::vector<std::vector<int64_t>> range_idx = {{15,20},{36,40},{56,60},{78,80},{40,48},{60,67},{80,86},
                                            		{100,105},{120,124},{140,143},{151,160},{171,180},{191,200},
                                            		{211,214},{217,220},{231,234},{237,240},{220,226},{240,246},
                                            		{260,266},{327,331},{346,352},{366,372},{387,391}};
    IpasDemo ipasdemo(&nh, tasks_data, agents, comm, num_vehicle, num_tasks, num_sensors, num_row, num_col, range_idx);
    	
    ROS_INFO("Initializing ipas demo...");
    // ros::Rate loop_rate(0.5); // 5Hz
    // while (ros::ok()) {
    //     loop_rate.sleep();
    //     ros::spinOnce();
    // }
    ros::spin();
    return 0;
}