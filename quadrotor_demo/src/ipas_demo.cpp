#include "../include/quadrotor_demo/ipas_demo.h"

IpasDemo::IpasDemo(ros::NodeHandle* nodehandle, std::vector<Task>& tasks_data, std::vector<AutoVehicle>& agent, 
		Eigen::MatrixXi& comm, int64_t num_vehicle, int64_t num_tasks, int64_t num_sensors, int64_t num_row, 
			int64_t num_col, std::vector<std::vector<int64_t>> range_idx) 
			: nh_(*nodehandle), tasks_data_(tasks_data), agents_(agent), comm_(comm), num_vehicle_(num_vehicle), 
			num_tasks_(num_tasks), num_sensors_(num_sensors), num_row_(num_row), num_col_(num_col), range_idx_
			(range_idx) { 
	// constructor
 	// ROS_INFO("In class constructor of IpasDemo");
	assert(num_sensors_ == 4 && "The number of sensors in this demo must be strictly equal to 4!");
	init();
    initSub();
    initPub();
    // file_path.open("/home/han/quadrotordemo_ws/src/ipas.txt",std::ios::trunc);
}

IpasDemo::~IpasDemo() {

}

void IpasDemo::initMap() {
    uncertain_grid = GridGraph::CreateSquareGrid(num_row_,num_col_,1,vehicle_team_,tasks_);
	uncertain_graph = GridGraph::BuildGraphFromSquareGrid(uncertain_grid, true);

    // Build true map
    true_grid = GridGraph::CreateSquareGrid(num_row_,num_col_,1);
    // for(auto rg: range_idx_){
    //     for(int ii = rg[0]; ii<rg[1];ii++){
    //         true_grid->SetObstacleRegionLabel(ii,1);
    //     }
    // }
    // true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);
    //===============================================================================================//
    //============================================= CBBA ============================================//
    //===============================================================================================//   
    // Initialize local_grid and local_graph
    IPASMeasurement::InitLocalGraph(vehicle_team_,uncertain_grid);  

}


void IpasDemo::init() {
	updatemap_flag = true;
	updategraph_flag = false;
	ipas_tt = 0;
	index2Id = {{0,+num_col_-1}, {1,num_col_}, {2, +num_col_+1},
				{3,-1}, {4,0}, {5,1},
				{6,-num_col_-1}, {7,-num_col_}, {8,-num_col_+1}};
	tasks_ = TasksSet(tasks_data_);
	vehicle_team_ = IPASMeasurement::ConstructAutoTeam(agents_);
	getTaskId();
	initMap();
}


void IpasDemo::initSub() {
    // ROS_INFO("Initializing Subscribers");  
    currentpos1_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &IpasDemo::currentpos1Callback,this);
    currentpos2_sub_ = nh_.subscribe("/drone2/ground_truth_to_tf/pose", 1, &IpasDemo::currentpos2Callback,this);
    currentpos3_sub_ = nh_.subscribe("/drone3/ground_truth_to_tf/pose", 1, &IpasDemo::currentpos3Callback,this);
    currentpos4_sub_ = nh_.subscribe("/drone4/ground_truth_to_tf/pose", 1, &IpasDemo::currentpos4Callback,this);
    localmap1_sub_ = nh_.subscribe("/localmap1", 1, &IpasDemo::localmap1Callback,this);
    localmap2_sub_ = nh_.subscribe("/localmap2", 1, &IpasDemo::localmap2Callback,this);
    localmap3_sub_ = nh_.subscribe("/localmap3", 1, &IpasDemo::localmap3Callback,this);
    localmap4_sub_ = nh_.subscribe("/localmap4", 1, &IpasDemo::localmap4Callback,this);
    updatemapflag_sub_ = nh_.subscribe("/updatemap_flag", 1, &IpasDemo::updatemapflagCallback,this);
    updategraphflag_sub_ = nh_.subscribe("/updategraph_flag", 1, &IpasDemo::updategraphflagCallback,this);
}

void IpasDemo::initPub() {
    // ROS_INFO("Initializing Publishers");
    task_pub_ = nh_.advertise<quadrotor_demo::final_path>("/sensor_path", 1, true); 
    iteration_complete_pub_ = nh_.advertise<std_msgs::Bool>("/updatemap_flag", 1, true); 
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_axis", 10);
    markerarray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_finalpath", 10);
    obsarray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_obstacle", 10);
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

void IpasDemo::currentpos4Callback(const geometry_msgs::PoseStamped& odom4) {   

    // posvector[2][0] = odom3.pose.position.x;
    // posvector[2][1] = odom3.pose.position.y;
    // posvector[2][2] = odom3.pose.position.z;
    // posvector[2][3] = odom3.pose.orientation.x; 
    // posvector[2][4] = odom3.pose.orientation.y;
    // posvector[2][5] = odom3.pose.orientation.z;
    // posvector[2][6] = odom3.pose.orientation.w;
}

void IpasDemo::localmap1Callback(const quadrotor_demo::localmap& localmap1) {
	updateLocalmap(localmap1);
}

void IpasDemo::localmap2Callback(const quadrotor_demo::localmap& localmap2) {
	updateLocalmap(localmap2);
}

void IpasDemo::localmap3Callback(const quadrotor_demo::localmap& localmap3) {
	updateLocalmap(localmap3);
}

void IpasDemo::localmap4Callback(const quadrotor_demo::localmap& localmap4) {
    updateLocalmap(localmap4);
}

void IpasDemo::updateLocalmap(const quadrotor_demo::localmap& localmap) {
	int32_t x_col = int32_t(localmap.xpos);
	int32_t y_row = int32_t(localmap.ypos);
	int64_t id = true_grid->GetIDFromCoordinate(x_col, y_row);
    // std::shared_ptr<SquareGrid> new_grid = GridGraph::CreateSquareGrid(num_row_,num_col_,1);
	for(std::vector<quadrotor_demo::obstacle_info>::const_iterator itr = localmap.obstacle_data.begin(); 
			itr != localmap.obstacle_data.end(); ++itr) {
		if (itr->isobstacle) {
			int64_t neighborId = id + index2Id[itr->id];
			// check if the id is within the boundry of the map
			// check the up and bottom boundry
			if (neighborId < 0 || neighborId >=  num_col_ * num_row_) {
				continue;
			}
			// check the left boundry
			if (id % num_col_ == 0 && neighborId == id - 1) {
				continue;
			}
			// check the right boundry
			if ((id+1) % num_col_ == 0 && neighborId == id + 1) {
				continue;
			}
			obstacles.insert(neighborId);
            true_grid->SetObstacleRegionLabel(neighborId,1);
		}
	}
	// std::shared_ptr<SquareGrid> new_grid = GridGraph::CreateSquareGrid(num_row_,num_col_,1);
}

void IpasDemo::updatemapflagCallback(const std_msgs::Bool& flag_msg) {
    updatemap_flag = flag_msg.data;
    printAxis();
    if (updatemap_flag == true) {
        ros::Rate loop_rate(5);
        loop_rate.sleep();
    	mobilePath();
    }
}

void IpasDemo::updategraphflagCallback(const std_msgs::Bool& graphFlag_msg) {
    updategraph_flag = graphFlag_msg.data;
    if (updategraph_flag == true) {
    	true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);

    	// update sensor pose
        ros::Rate loop_rate(5);
        loop_rate.sleep();
    	updateSensorPos();
        IPASMeasurement::UpdateLocalMap(vehicle_team_,true_graph,sensing_tasks_);
        IPASMeasurement::MergeLocalMap(vehicle_team_);

        std::cout << "Localmap Updated!" << std::endl;
        std_msgs::Bool iterationComplete_flag;
        iterationComplete_flag.data = true;
        iteration_complete_pub_.publish(iterationComplete_flag);
        std::cout << "Iteration " << ipas_tt << " finished!" << std::endl;
        std::cout << "======================================" << std::endl;
        std::cout << "======================================" << std::endl;

    }
    
}

void IpasDemo::updateSensorPos() {
	for (auto agentele : vehicle_team_->auto_team_) {
		if (agentele->vehicle_type_ == TaskType::MEASURE && !agentele->task_path_.empty()) {
			Task tsk = sensing_tasks_.GetTaskFromID(agentele->task_path_.back());
			agentele->pos_ = tsk.pos_.front();
		}
	}
}

void IpasDemo::mobilePath() {
	ipas_tt ++;
	std::cout << "======================================" << std::endl;
	std::cout << "======================================" << std::endl;
	std::cout << "Iteration: " << ipas_tt << std::endl;

    std::cout << "Obstacle id: ";
    for (auto itr = obstacles.begin(); itr != obstacles.end(); itr++) {
        std::cout << *itr << ", "; 
    }
    std::cout << std::endl;

    CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);

    // Compute the path for the auto team while satisfying its local assignment
    std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
    //=============================================================//
    //============================== DEBUG ========================//
    //=============================================================//
    for(auto p: path_ltl_){
    	if (p.first < num_vehicle_) {
	        std::cout << "The path for vehicle " << p.first << " is: ";
	        // file_path << "The path for vehicle " << p.first << " is: ";
	        for(auto v: p.second){
	            std::cout << v->id_ <<", ";
	            // file_path << v->id_ <<", ";
	        }
	        std::cout << std::endl;
	        // file_path << "\n";
    	}
    }
    //=============================================================//
    //=============================================================//
    //=============================================================//
    
    // Check whether the IPAS convergence is achieved
    bool flag_IPAS = IPASMeasurement::IPASConvergence(vehicle_team_,path_ltl_);
    if (flag_IPAS == true) {
    	std::cout << "The required iteration is " << ipas_tt <<std::endl; 
    	printObstacle();
    	printValidPath(path_ltl_);
    	// file_path.close();
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

    sensing_tasks_ = IPASMeasurement::ConstructMeasurementTasks(vehicle_team_);
    hotspots_.clear();
    hotspots_ = sensing_tasks_.GetHotspots();
    // std::cout << "hotspots " << hotspots_.size() << std::endl;
    printHotspots(hotspots_);
    hotspotIndex.clear();

    hotspotIndex = CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);
    std::map<int64_t,Path_t<SquareCell*>> path_sensing_ = IPASMeasurement::GeneratePaths(vehicle_team_,sensing_tasks_,TaskType::MEASURE);
    for(auto p: path_sensing_){
    	if (p.first >= (num_vehicle_ - num_sensors_)) {
	        std::cout << "The path for sensor " << p.first << " is: ";
	        // file_path << "The path for vehicle " << p.first << " is: ";
	        for(auto v: p.second){
	            std::cout << v->id_ <<", ";
	            // file_path << v->id_ <<", ";
	        }
	        std::cout << std::endl;
	        // file_path << "\n";
    	}
    }
    std::cout << "publishing path for sensors" << std::endl;
    pathesPub(path_sensing_);
}

void IpasDemo::pathesPub(const std::map<int64_t,Path_t<SquareCell*>>& pathes) {

	quadrotor_demo::final_path finalpath_msg;
	quadrotor_demo::path path_msg_empty;

    for(auto p: pathes){
    	if (p.first >= (num_vehicle_ - num_sensors_)) {
    		quadrotor_demo::pathes pathes_msg;
        	pathes_msg.path_name = std::string("Drone ") + std::string(std::to_string(p.first - (num_vehicle_ - num_sensors_) + 1));
        	if (p.second.empty()) {
        		pathes_msg.empty = true;
        		int cnt = num_sensors_;
        		while (cnt > 0) {
        			pathes_msg.pathes_data.push_back(path_msg_empty);
        			cnt--;
        		}
		        finalpath_msg.final_path.push_back(pathes_msg);
        	} else {
        		pathes_msg.empty = false;
	    		quadrotor_demo::path path_msg;
	    		int cnt = 0;
		        for(auto v: p.second){
                    int pathId = v->id_;
                    bool found = true;
                    if (found) {
                        quadrotor_demo::pose pose_msg;
                        pose_msg.x = v->position_.x;
                        pose_msg.y = v->position_.y;
                        pose_msg.id = v->id_;
                        pose_msg.xcoordinate = v->coordinate_.x;
                        pose_msg.ycoordinate = v->coordinate_.y;
                        path_msg.path.push_back(pose_msg);
                        bool containHotspot = false;
                        for (int i = 0; i < hotspotIndex[p.first].size(); i++) {
                            if (hotspots_[hotspotIndex[p.first][i]] == v->id_) {
                                containHotspot = true;
                            }
                        }
                        if (containHotspot) {
                            cnt++;
                            pathes_msg.pathes_data.push_back(path_msg);
                            if (v->id_ != p.second.back()->id_) {
                                path_msg.path.clear();
                            } else {
                                continue;
                            }
                        }
                    }
		        	
		        }
		        while (cnt < num_sensors_) {
	        		pathes_msg.pathes_data.push_back(path_msg_empty);
	        		cnt++;
		        }
		        finalpath_msg.final_path.push_back(pathes_msg);
        	}        	
    	}
    }
    task_pub_.publish(finalpath_msg);
}

void IpasDemo::getTaskId() {
	for (auto tks : tasks_data_) {
		task_id_.insert(tks.pos_[0]);
	}
}

void IpasDemo::printValidPath(std::map<int64_t,Path_t<SquareCell*>>& validPath) {

	visualization_msgs::MarkerArray path_marker_array;

    visualization_msgs::Marker path_marker;
    path_marker.type = visualization_msgs::Marker::CUBE_LIST;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.ns = "paths";
    path_marker.scale.x = 1;
    path_marker.scale.y = 1;
    path_marker.scale.z = 0.1;	
    path_marker.header.frame_id = "/world";
    path_marker.color.a = 1.0; 
    path_marker.color.g = 1.0;
    path_marker.id = 0;
    path_marker.lifetime = ros::Duration();
    // ros::Duration lifetime;
    // arrow_marker.lifetime = lifetime.fromSec(0.04); // lifetime of 40ms : 25Hz

	for(auto p: validPath){
    	if (p.first < (num_vehicle_ - num_sensors_)) {
	        for(auto v: p.second){
				geometry_msgs::Point obj;
		        obj.x = v->position_.x;
		        obj.y = v->position_.y;
		        // std::cout << "pos: " << obj.x << " " << obj.y << std::endl;
		        obj.z = 0.1;
		        path_marker.points.push_back(obj);
		        if (task_id_.count(v->id_)) {
		        	path_marker.color.g = 0.0;
		        	path_marker.color.r = 1.0;

		        } else {
		        	path_marker.color.g = 1.0;
		        	path_marker.color.r = 0.0;
		        }
		        path_marker.colors.push_back(path_marker.color);
		        path_marker.header.stamp = ros::Time::now();
		        path_marker_array.markers.push_back(path_marker);
    		}
    	}
    }
    markerarray_pub_.publish(path_marker_array);
}


void IpasDemo::printHotspots(std::vector<int64_t>& hotspots) {
	visualization_msgs::MarkerArray hotspot_marker_array;

    visualization_msgs::Marker hotspot_marker;
    hotspot_marker.type = visualization_msgs::Marker::CUBE_LIST;
    hotspot_marker.action = visualization_msgs::Marker::ADD;
    hotspot_marker.ns = "hotspots";
    hotspot_marker.scale.x = 1;
    hotspot_marker.scale.y = 1;
    hotspot_marker.scale.z = 0.1;
    hotspot_marker.header.frame_id = "/world";
    hotspot_marker.color.a = 1.0; 
    hotspot_marker.color.r = 1.0;
    hotspot_marker.id = 0;
    ros::Duration lifetime;
    hotspot_marker.lifetime = lifetime.fromSec(1); // lifetime of 40ms : 25Hz

	for(auto itr = hotspots.begin(); itr != hotspots.end(); itr++){
			geometry_msgs::Point obj;
	        obj.x = *itr % num_col_ + 0.5;
	        obj.y = *itr / num_row_ + 0.5;
	        obj.z = 0.1;
	        hotspot_marker.points.push_back(obj);
	        hotspot_marker.colors.push_back(hotspot_marker.color);
	        hotspot_marker.header.stamp = ros::Time::now();
	        hotspot_marker_array.markers.push_back(hotspot_marker);
    }
    markerarray_pub_.publish(hotspot_marker_array);
}

void IpasDemo::printObstacle() {
	visualization_msgs::MarkerArray obstacle_marker_array;

    visualization_msgs::Marker obstacle_marker;
    obstacle_marker.type = visualization_msgs::Marker::CUBE_LIST;
    obstacle_marker.action = visualization_msgs::Marker::ADD;
    obstacle_marker.ns = "obstacle";
    obstacle_marker.scale.x = 1;
    obstacle_marker.scale.y = 1;
    obstacle_marker.scale.z = 1;
    obstacle_marker.header.frame_id = "/world";
    obstacle_marker.color.a = 1.0; 
    obstacle_marker.color.g = 0.0;
    obstacle_marker.color.b = 1.0;
    obstacle_marker.color.r = 0.0;
    obstacle_marker.id = 0;
    obstacle_marker.lifetime = ros::Duration();

	for(auto range_id : range_idx_) {
		for (auto id = range_id[0]; id < range_id[1]; id++) {
			geometry_msgs::Point obj;
	        obj.x = id % num_col_ + 0.5;
	        obj.y = id / num_row_ + 0.5;
	        // std::cout << "pos: " << obj.x << " " << obj.y << std::endl;
	        obj.z = 0.1;
	        obstacle_marker.points.push_back(obj);
	        obstacle_marker.colors.push_back(obstacle_marker.color);
	        obstacle_marker.header.stamp = ros::Time::now();
	        obstacle_marker_array.markers.push_back(obstacle_marker);
	    }
    }
    obsarray_pub_.publish(obstacle_marker_array);
}

void IpasDemo::printAxis() {
	// x axis
	visualization_msgs::Marker arrow_marker_x;
	arrow_marker_x.type = visualization_msgs::Marker::ARROW;
	arrow_marker_x.action = visualization_msgs::Marker::ADD;	
	arrow_marker_x.ns = "axis";
	arrow_marker_x.scale.x=0.1;
	arrow_marker_x.scale.y=0.2;
	arrow_marker_x.scale.z = 0.5;
	arrow_marker_x.header.frame_id = "/world";
	arrow_marker_x.color.a = 1.0;
	arrow_marker_x.color.r = 1.0;
	arrow_marker_x.id = 1;
	arrow_marker_x.lifetime = ros::Duration();

	geometry_msgs::Point startPoint1;
    startPoint1.x = -1;
    startPoint1.y = -1;
    startPoint1.z = 0.1;
    arrow_marker_x.points.push_back(startPoint1);
    arrow_marker_x.colors.push_back(arrow_marker_x.color);
    geometry_msgs::Point endPoint1;
    endPoint1.x = 1;
    endPoint1.y = -1;
    endPoint1.z = 0.1;
    arrow_marker_x.points.push_back(endPoint1);
    arrow_marker_x.colors.push_back(arrow_marker_x.color);
    arrow_marker_x.header.stamp = ros::Time::now(); 
    marker_pub_.publish(arrow_marker_x);
    // y axis
	visualization_msgs::Marker arrow_marker_y;
	arrow_marker_y.type = visualization_msgs::Marker::ARROW;
	arrow_marker_y.action = visualization_msgs::Marker::ADD;	
	arrow_marker_y.ns = "axis";
	arrow_marker_y.scale.x=0.15;
	arrow_marker_y.scale.y=0.3;
	arrow_marker_y.scale.z = 0.5;
	arrow_marker_y.header.frame_id = "/world";
	arrow_marker_y.color.a = 1.0;
	arrow_marker_y.color.g = 1.0;
	arrow_marker_y.id = 2;
	arrow_marker_y.lifetime = ros::Duration();
    geometry_msgs::Point startPoint2;
    startPoint2.x = -1;
    startPoint2.y = -1;
    startPoint2.z = 0.1;
    arrow_marker_y.points.push_back(startPoint2);
    arrow_marker_y.colors.push_back(arrow_marker_y.color);
    geometry_msgs::Point endPoint2;
    endPoint2.x = -1;
    endPoint2.y = 1;
    endPoint2.z = 0.1;
    arrow_marker_y.points.push_back(endPoint2);
    arrow_marker_y.colors.push_back(arrow_marker_y.color);

	arrow_marker_y.header.stamp = ros::Time::now(); 
    marker_pub_.publish(arrow_marker_y);

    // x text
    visualization_msgs::Marker textx_marker;
	textx_marker.header.frame_id = "/world";
	textx_marker.header.stamp = ros::Time::now();
	textx_marker.ns = "text";
	textx_marker.id = 3;
	textx_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	textx_marker.action = visualization_msgs::Marker::ADD;

	textx_marker.pose.position.x = 0.0;
	textx_marker.pose.position.y = -2.0;
	textx_marker.pose.position.z = 0.1;
	textx_marker.pose.orientation.x = 0.0;
	textx_marker.pose.orientation.y = 0.0;
	textx_marker.pose.orientation.z = 0.0;
	textx_marker.pose.orientation.w = 1.0;

	textx_marker.text = "X AXIS";

	textx_marker.scale.z = 0.5;

	textx_marker.color.r = 1.0;
	textx_marker.color.a = 1.0;
	marker_pub_.publish(textx_marker);

    // y text
    visualization_msgs::Marker texty_marker;
	texty_marker.header.frame_id = "/world";
	texty_marker.header.stamp = ros::Time::now();
	texty_marker.ns = "text";
	texty_marker.id = 4;
	texty_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	texty_marker.action = visualization_msgs::Marker::ADD;

	texty_marker.pose.position.x = -2.0;
	texty_marker.pose.position.y = 0.0;
	texty_marker.pose.position.z = 0.1;
	texty_marker.pose.orientation.x = 0.0;
	texty_marker.pose.orientation.y = 0.0;
	texty_marker.pose.orientation.z = 0.0;
	texty_marker.pose.orientation.w = 1.0;

	texty_marker.text = "Y AXIS";

	texty_marker.scale.z = 0.5;

	texty_marker.color.g = 1.0;
	texty_marker.color.a = 1.0;
	marker_pub_.publish(texty_marker);

    visualization_msgs::Marker text0_marker;
	text0_marker.header.frame_id = "/world";
	text0_marker.header.stamp = ros::Time::now();
	text0_marker.ns = "text";
	text0_marker.id = 5;
	text0_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text0_marker.action = visualization_msgs::Marker::ADD;

	text0_marker.pose.position.x = 0.5;
	text0_marker.pose.position.y = -0.5;
	text0_marker.pose.position.z = 0.1;
	text0_marker.pose.orientation.x = 0.0;
	text0_marker.pose.orientation.y = 0.0;
	text0_marker.pose.orientation.z = 0.0;
	text0_marker.pose.orientation.w = 1.0;

	text0_marker.text = "0";

	text0_marker.scale.z = 0.7;

	text0_marker.color.r = 1.0;
	text0_marker.color.g = 1.0;
	text0_marker.color.b = 0.0;
	text0_marker.color.a = 1.0;
	marker_pub_.publish(text0_marker);

    visualization_msgs::Marker text0a_marker;
    text0a_marker.header.frame_id = "/world";
    text0a_marker.header.stamp = ros::Time::now();
    text0a_marker.ns = "text";
    text0a_marker.id = 10;
    text0a_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text0a_marker.action = visualization_msgs::Marker::ADD;

    text0a_marker.pose.position.x = 0.5;
    text0a_marker.pose.position.y = -0.5;
    text0a_marker.pose.position.z = 0.1;
    text0a_marker.pose.orientation.x = 0.0;
    text0a_marker.pose.orientation.y = 0.0;
    text0a_marker.pose.orientation.z = 0.0;
    text0a_marker.pose.orientation.w = 1.0;

    text0a_marker.text = "0";

    text0a_marker.scale.z = 0.7;

    text0a_marker.color.r = 1.0;
    text0a_marker.color.g = 1.0;
    text0a_marker.color.b = 0.0;
    text0a_marker.color.a = 1.0;
    marker_pub_.publish(text0a_marker);

    visualization_msgs::Marker text14_marker;
	text14_marker.header.frame_id = "/world";
	text14_marker.header.stamp = ros::Time::now();
	text14_marker.ns = "text";
	text14_marker.id = 6;
	text14_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text14_marker.action = visualization_msgs::Marker::ADD;

	text14_marker.pose.position.x = 14.5;
	text14_marker.pose.position.y = -0.5;
	text14_marker.pose.position.z = 0.1;
	text14_marker.pose.orientation.x = 0.0;
	text14_marker.pose.orientation.y = 0.0;
	text14_marker.pose.orientation.z = 0.0;
	text14_marker.pose.orientation.w = 1.0;

	text14_marker.text = "14";

	text14_marker.scale.z = 0.7;

	text14_marker.color.r = 1.0;
	text14_marker.color.g = 1.0;
	text14_marker.color.b = 0.0;
	text14_marker.color.a = 1.0;
	marker_pub_.publish(text14_marker);

    visualization_msgs::Marker text30_marker;
	text30_marker.header.frame_id = "/world";
	text30_marker.header.stamp = ros::Time::now();
	text30_marker.ns = "text";
	text30_marker.id = 7;
	text30_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text30_marker.action = visualization_msgs::Marker::ADD;

	text30_marker.pose.position.x = 29.5;
	text30_marker.pose.position.y = -0.5;
	text30_marker.pose.position.z = 0.1;
	text30_marker.pose.orientation.x = 0.0;
	text30_marker.pose.orientation.y = 0.0;
	text30_marker.pose.orientation.z = 0.0;
	text30_marker.pose.orientation.w = 1.0;

	text30_marker.text = "29";

	text30_marker.scale.z = 0.7;

	text30_marker.color.r = 1.0;
	text30_marker.color.g = 1.0;
	text30_marker.color.b = 0.0;
	text30_marker.color.a = 1.0;
	marker_pub_.publish(text30_marker);

    visualization_msgs::Marker text14a_marker;
	text14a_marker.header.frame_id = "/world";
	text14a_marker.header.stamp = ros::Time::now();
	text14a_marker.ns = "text";
	text14a_marker.id = 8;
	text14a_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text14a_marker.action = visualization_msgs::Marker::ADD;

	text14a_marker.pose.position.x = -0.5;
	text14a_marker.pose.position.y = 14.5;
	text14a_marker.pose.position.z = 0.1;
	text14a_marker.pose.orientation.x = 0.0;
	text14a_marker.pose.orientation.y = 0.0;
	text14a_marker.pose.orientation.z = 0.0;
	text14a_marker.pose.orientation.w = 1.0;

	text14a_marker.text = "14";

	text14a_marker.scale.z = 0.7;

	text14a_marker.color.r = 1.0;
	text14a_marker.color.g = 1.0;
	text14a_marker.color.b = 0.0;
	text14a_marker.color.a = 1.0;

	marker_pub_.publish(text14a_marker);

    visualization_msgs::Marker text30a_marker;
    text30a_marker.header.frame_id = "/world";
    text30a_marker.header.stamp = ros::Time::now();
    text30a_marker.ns = "text";
    text30a_marker.id = 9;
    text30a_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text30a_marker.action = visualization_msgs::Marker::ADD;

    text30a_marker.pose.position.x = -0.5;
    text30a_marker.pose.position.y = 29.5;
    text30a_marker.pose.position.z = 0.1;
    text30a_marker.pose.orientation.x = 0.0;
    text30a_marker.pose.orientation.y = 0.0;
    text30a_marker.pose.orientation.z = 0.0;
    text30a_marker.pose.orientation.w = 1.0;

    text30a_marker.text = "29";

    text30a_marker.scale.z = 0.7;

    text30a_marker.color.r = 1.0;
    text30a_marker.color.g = 1.0;
    text30a_marker.color.b = 0.0;
    text30a_marker.color.a = 1.0;

    marker_pub_.publish(text30a_marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ipasdemo"); //node name
    ros::NodeHandle nh; 


    //===============================================================================================//
    //===========================================Task - Agent========================================//
    //===============================================================================================//
    int64_t num_vehicle = 8;
    int64_t num_tasks = 10;
    // Tasks 
    // Define the task information:
    // Index of task, AP value(ltl), Position, Task Type, Number of vehicle
    std::vector<Task> tasks_data = {Task(0,2,{880},TaskType::RESCUE,num_vehicle),
                                    Task(1,3,{457},TaskType::RESCUE,num_vehicle),
                                    Task(2,4,{194},TaskType::RESCUE,num_vehicle),
                                    Task(3,5,{108},TaskType::RESCUE,num_vehicle),
                                    Task(4,6,{145},TaskType::RESCUE,num_vehicle),
                                    Task(5,7,{356},TaskType::RESCUE,num_vehicle),
                                    Task(6,8,{290},TaskType::RESCUE,num_vehicle),
                                    Task(7,9,{505},TaskType::RESCUE,num_vehicle),
                                    Task(8,10,{565},TaskType::RESCUE,num_vehicle),
                                    Task(9,11,{865},TaskType::RESCUE,num_vehicle)};
    // Auto Vehicle Team
    // Index of drone, Initial position, # of drones, Communicate network, Task Type, # of tasks
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,num_vehicle);
    int64_t num_sensors = 4;
    std::vector<AutoVehicle> agents = {AutoVehicle(0,0,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    AutoVehicle(1,29,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    AutoVehicle(2,870,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    AutoVehicle(3,899,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    AutoVehicle(4,0,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    AutoVehicle(5,29,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    AutoVehicle(6,870,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    AutoVehicle(7,899,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors)};

    int64_t num_row = 30;
    int64_t num_col = 30;

    // obstacles range for visulization
    std::vector<std::vector<int64_t>> range_idx = {{4,6},{11,12},{26,29},{34,38},{40,42},{56,59},{64,68},{70,71},{86,89},
                                                {95,98},{116,119},{126,128},{147,148},{156,158},{167,169},{177,178},{187,189},
                                                {196,199},{217,219},{222,227},{240,243},{248,250},{253,256},
                                                {270,273},{313,316},{322,323},{344,348},{352,355},{363,367},{372,379},
                                                {384,390},{393,397},{402,409},{414,420},{423,424},{433,442},{450,451},{453,454},{459,473},
                                                {481,482},{483,485},{491,503},{511,516},{526,534},{542,547},{556,564},
                                                {568,570},{573,575},{576,577},{586,587},{589,594},{596,600},{604,605},{606,608},{615,617},
                                                {621,624},{626,630},{637,638},{656,660},{667,668},{687,690},{697,699},{701,703},
                                                {718,720},{728,729},{731,733},{758,759},{761,766},{793,795},{823,825},{890,893}};

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