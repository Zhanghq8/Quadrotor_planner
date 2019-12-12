#include "../include/quadrotor_demo/ipas_demo.h"

IpasDemo::IpasDemo(ros::NodeHandle* nodehandle, std::vector<Task>& tasks_data, std::vector<AutoVehicle>& agent, 
		Eigen::MatrixXi& comm, int64_t num_vehicle, int64_t num_tasks, int64_t num_sensors, int64_t num_row, 
			int64_t num_col, std::vector<std::vector<int64_t>> range_idx) 
			: nh_(*nodehandle), tasks_data_(tasks_data), agents_(agent), comm_(comm), num_vehicle_(num_vehicle), 
			num_tasks_(num_tasks), num_sensors_(num_sensors), num_row_(num_row), num_col_(num_col), range_idx_
			(range_idx) { 
	// constructor
 	// ROS_INFO("In class constructor of IpasDemo");
	assert(num_sensors_ == 3 && "The number of sensors in this demo must be strictly equal to 3!");
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
    true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);
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
    localmap1_sub_ = nh_.subscribe("/localmap1", 1, &IpasDemo::localmap1Callback,this);
    localmap2_sub_ = nh_.subscribe("/localmap2", 1, &IpasDemo::localmap2Callback,this);
    localmap3_sub_ = nh_.subscribe("/localmap3", 1, &IpasDemo::localmap3Callback,this);
    updatemapflag_sub_ = nh_.subscribe("/updatemap_flag", 1, &IpasDemo::updatemapflagCallback,this);
    updategraphflag_sub_ = nh_.subscribe("/updategraph_flag", 1, &IpasDemo::updategraphflagCallback,this);
}

void IpasDemo::initPub() {
    // ROS_INFO("Initializing Publishers");
    task_pub_ = nh_.advertise<quadrotor_demo::final_path>("/sensor_path", 1, true); 
    iteration_complete_pub_ = nh_.advertise<std_msgs::Bool>("/updatemap_flag", 1, true); 
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    markerarray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_markerarray", 10);
    obsarray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_markerarray1", 10);
    

    // control1input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
    // control2input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone2/cmd_vel", 1, true);
    // control3input_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone3/cmd_vel", 1, true);
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

void IpasDemo::localmap1Callback(const quadrotor_demo::localmap& localmap1) {
	updateLocalmap(localmap1);
}

void IpasDemo::localmap2Callback(const quadrotor_demo::localmap& localmap2) {
	updateLocalmap(localmap2);
}

void IpasDemo::localmap3Callback(const quadrotor_demo::localmap& localmap3) {
	updateLocalmap(localmap3);
}

void IpasDemo::updateLocalmap(const quadrotor_demo::localmap& localmap) {
	int32_t x_col = int32_t(localmap.xpos);
	int32_t y_row = int32_t(localmap.ypos);
	int64_t id = true_grid->GetIDFromCoordinate(x_col, y_row);
	for(std::vector<quadrotor_demo::obstacle_info>::const_iterator itr = localmap.obstacle_data.begin(); 
			itr != localmap.obstacle_data.end(); ++itr) {
		if (itr->isobstacle) {
			int64_t neighborId = id + index2Id[itr->id];
			// check if the id is within the boundry of the map
			// check the up and bottom boundry
			if (neighborId < 0 || neighborId > num_col_ * num_row_) {
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
			// std::cout << "!!!!!!!!!!!!!!!!!!!!!!Debug!!!!!!!!!!!!!!!!!!!!!" << std::endl; 
			// std::cout << "id: " << itr->id << " neighborid: " << neighborId << std::endl; 
			// std::cout << "!!!!!!!!!!!!!!!!!!!!!!Debug!!!!!!!!!!!!!!!!!!!!!" << std::endl; 
			obstacles.insert(neighborId);
		}
	}
	std::shared_ptr<SquareGrid> new_grid = GridGraph::CreateSquareGrid(num_row_,num_col_,1);
	for (auto itr = obstacles.begin(); itr != obstacles.end(); itr++) {
		new_grid->SetObstacleRegionLabel(*itr,1);
	}
	true_grid = new_grid;
}

void IpasDemo::updatemapflagCallback(const std_msgs::Bool& flag_msg) {
    updatemap_flag = flag_msg.data;
    printAxis();
    if (updatemap_flag == true) {
    	mobilePath();
    }
}

void IpasDemo::updategraphflagCallback(const std_msgs::Bool& graphFlag_msg) {
    updategraph_flag = graphFlag_msg.data;
    if (updategraph_flag == true) {
    	true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false);

    	// update sensor pose
    	updateSensorPos();
        IPASMeasurement::UpdateLocalMap(vehicle_team_,true_graph,sensing_tasks_);
        IPASMeasurement::MergeLocalMap(vehicle_team_);


   //      int cnt = 0;
   //      file_path << "=============================================" << "\n";
   //      file_path << "=============================================" << "\n";
   //      for (auto element : vehicle_team_->auto_team_) {
   //      	file_path << "local graph " <<cnt << ": "<<  "\n";
   //      	std::shared_ptr<Graph_t<SquareCell*>> new_graph = element->local_graph_;
	  //   	std::vector<Vertex_t<SquareCell*>*> vts = new_graph->GetAllVertex();
			// for(auto vt: vts){

			//     file_path << "Vertex " << vt->state_->id_ << ": "<<  "\n";
			//     file_path << "The probability p is " << vt->state_->p_ << "\n";
			//     file_path << "The IG is " << vt->state_->ig_ << "\n";

			//     file_path << "The neighbors are: " << "\n";
			//     std::vector<Vertex_t<SquareCell*>*> neighbs = vt->GetNeighbours();
			//     for(auto nb: neighbs){
			//         auto ecost = vt->GetEdgeCost(nb);
			//         file_path << "Vertex " << nb->state_->id_ << ". The edge cost is: " << ecost << "\n";
			//         file_path << "The probability p is " << vt->state_->p_ << "\n";
			//     	file_path << "The IG is " << vt->state_->ig_ << "\n";
			//     }
			//     file_path << "=============================================" << "\n";
			//     file_path << "=============================================" << "\n";
			// }   
   //      	cnt++;
   //      }

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
	// file_path << "Iteration: " << ipas_tt << "\n";
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
    printHotspots(hotspots_);
    CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);

    std::map<int64_t,Path_t<SquareCell*>> path_sensing_ = IPASMeasurement::GeneratePaths(vehicle_team_,sensing_tasks_,TaskType::MEASURE);
    for(auto p: path_sensing_){
    	if (p.first >= 3) {
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
    pathesPub(path_sensing_);
}

void IpasDemo::pathesPub(const std::map<int64_t,Path_t<SquareCell*>>& pathes) {
	quadrotor_demo::final_path finalpath_msg;
	quadrotor_demo::path path_msg_empty;
	// int path_size = pathes.size();
    for(auto p: pathes){
    	if (p.first >= 3) {
    		quadrotor_demo::pathes pathes_msg;
        	pathes_msg.path_name = std::string("Drone ") + std::string(std::to_string(p.first-2));
        	if (p.second.empty()) {
        		pathes_msg.empty = true;
        		int cnt = 3;
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
		        	quadrotor_demo::pose pose_msg;
		        	pose_msg.x = v->position_.x;
		        	pose_msg.y = v->position_.y;
		        	pose_msg.id = v->id_;
		        	pose_msg.xcoordinate = v->coordinate_.x;
		        	pose_msg.ycoordinate = v->coordinate_.y;
		        	path_msg.path.push_back(pose_msg);
		        	if (hotspots_.count(v->id_)) {
		        		cnt++;
		        		pathes_msg.pathes_data.push_back(path_msg);
		        		if (v->id_ != p.second.back()->id_) {
		        			path_msg.path.clear();
		        		} else {
		        			continue;
		        		}
		        	}
		        }
		        while (cnt < 3) {
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
    	if (p.first < 3) {
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


void IpasDemo::printHotspots(std::unordered_set<int64_t>& hotspots) {
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
	        obj.x = *itr % num_col_;
	        obj.y = *itr / num_row_;
	        // std::cout << "pos: " << obj.x << " " << obj.y << std::endl;
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
    obstacle_marker.ns = "obstcle";
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

    visualization_msgs::Marker textx0_marker;
	textx0_marker.header.frame_id = "/world";
	textx0_marker.header.stamp = ros::Time::now();
	textx0_marker.ns = "text";
	textx0_marker.id = 5;
	textx0_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	textx0_marker.action = visualization_msgs::Marker::ADD;

	textx0_marker.pose.position.x = 0.5;
	textx0_marker.pose.position.y = -0.5;
	textx0_marker.pose.position.z = 0.1;
	textx0_marker.pose.orientation.x = 0.0;
	textx0_marker.pose.orientation.y = 0.0;
	textx0_marker.pose.orientation.z = 0.0;
	textx0_marker.pose.orientation.w = 1.0;

	textx0_marker.text = "0";

	textx0_marker.scale.z = 0.7;

	textx0_marker.color.r = 1.0;
	textx0_marker.color.g = 1.0;
	textx0_marker.color.b = 0.0;
	textx0_marker.color.a = 1.0;
	marker_pub_.publish(textx0_marker);

    visualization_msgs::Marker textx14_marker;
	textx14_marker.header.frame_id = "/world";
	textx14_marker.header.stamp = ros::Time::now();
	textx14_marker.ns = "text";
	textx14_marker.id = 6;
	textx14_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	textx14_marker.action = visualization_msgs::Marker::ADD;

	textx14_marker.pose.position.x = 14.5;
	textx14_marker.pose.position.y = -0.5;
	textx14_marker.pose.position.z = 0.1;
	textx14_marker.pose.orientation.x = 0.0;
	textx14_marker.pose.orientation.y = 0.0;
	textx14_marker.pose.orientation.z = 0.0;
	textx14_marker.pose.orientation.w = 1.0;

	textx14_marker.text = "14";

	textx14_marker.scale.z = 0.7;

	textx14_marker.color.r = 1.0;
	textx14_marker.color.g = 1.0;
	textx14_marker.color.b = 0.0;
	textx14_marker.color.a = 1.0;
	marker_pub_.publish(textx14_marker);

    visualization_msgs::Marker texty0_marker;
	texty0_marker.header.frame_id = "/world";
	texty0_marker.header.stamp = ros::Time::now();
	texty0_marker.ns = "text";
	texty0_marker.id = 7;
	texty0_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	texty0_marker.action = visualization_msgs::Marker::ADD;

	texty0_marker.pose.position.x = -0.5;
	texty0_marker.pose.position.y = 0.5;
	texty0_marker.pose.position.z = 0.1;
	texty0_marker.pose.orientation.x = 0.0;
	texty0_marker.pose.orientation.y = 0.0;
	texty0_marker.pose.orientation.z = 0.0;
	texty0_marker.pose.orientation.w = 1.0;

	texty0_marker.text = "0";

	texty0_marker.scale.z = 0.7;

	texty0_marker.color.r = 1.0;
	texty0_marker.color.g = 1.0;
	texty0_marker.color.b = 0.0;
	texty0_marker.color.a = 1.0;
	marker_pub_.publish(texty0_marker);

    visualization_msgs::Marker texty14_marker;
	texty14_marker.header.frame_id = "/world";
	texty14_marker.header.stamp = ros::Time::now();
	texty14_marker.ns = "text";
	texty14_marker.id = 8;
	texty14_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	texty14_marker.action = visualization_msgs::Marker::ADD;

	texty14_marker.pose.position.x = -0.5;
	texty14_marker.pose.position.y = 14.5;
	texty14_marker.pose.position.z = 0.1;
	texty14_marker.pose.orientation.x = 0.0;
	texty14_marker.pose.orientation.y = 0.0;
	texty14_marker.pose.orientation.z = 0.0;
	texty14_marker.pose.orientation.w = 1.0;

	texty14_marker.text = "14";

	texty14_marker.scale.z = 0.7;

	texty14_marker.color.r = 1.0;
	texty14_marker.color.g = 1.0;
	texty14_marker.color.b = 0.0;
	texty14_marker.color.a = 1.0;

	marker_pub_.publish(texty14_marker);

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
                                	Task(1,3,{102},TaskType::RESCUE,num_vehicle),
                                	Task(2,4,{120},TaskType::RESCUE,num_vehicle),
                                	Task(3,5,{149},TaskType::RESCUE,num_vehicle),
                                	Task(4,6,{169},TaskType::RESCUE,num_vehicle),
                                	Task(5,7,{191},TaskType::RESCUE,num_vehicle)};
    // Auto Vehicle Team
    // Index of drone, Initial position, # of drones, Communicate network, Task Type, # of tasks
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,num_vehicle);
    int64_t num_sensors = 3;
    std::vector<AutoVehicle> agents = { AutoVehicle(0,0,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    	AutoVehicle(1,210,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    	AutoVehicle(2,14,num_vehicle,comm,TaskType::RESCUE,num_tasks,num_sensors),
                                    	AutoVehicle(3,0,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    	AutoVehicle(4,210,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors),
                                    	AutoVehicle(5,14,num_vehicle,comm,TaskType::MEASURE,num_tasks,num_sensors) };

    int64_t num_row = 15;
    int64_t num_col = 15;

    // obstacles range for visulization
    std::vector<std::vector<int64_t>> range_idx = {{6,9}, {20,24}, {35,39}, {76,79}, {90,94}, 
													{100,101}, {104,105}, {115,120}, {130,135}, 
													{150,153}, {165,170}, {180,185}, {193,195},
													{206,210}, {220,225}};

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