#include "runner/run_am_swarm.hpp"

AMSwarmX :: AMSwarmX(){

    traj_pub = nh.advertise<visualization_msgs::MarkerArray>("trajs", 1, true);
    startgoal_pub = nh.advertise<visualization_msgs::MarkerArray>("startgoals", 1, true);
    anchor_pub = nh.advertise<visualization_msgs::MarkerArray>("anchors", 1, true);
    path_pub = nh.advertise<visualization_msgs::MarkerArray>("paths", 1, true);
    gridpath_pub = nh.advertise<visualization_msgs::MarkerArray>("gridpaths", 1, true);
    griddata_pub = nh.advertise<visualization_msgs::MarkerArray>("griddatas", 1, true);
    boundary_pub = nh.advertise<visualization_msgs::MarkerArray>("boundaries", 1, true);
    convexpoly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    nonconvexsfc_pub = nh.advertise<visualization_msgs::MarkerArray>("nonconvex_sfc", 1, true);

    vel_x_pub = nh.advertise<std_msgs::Float64MultiArray>("vels_x", 1, true);
    vel_y_pub = nh.advertise<std_msgs::Float64MultiArray>("vels_y", 1, true);
    vel_z_pub = nh.advertise<std_msgs::Float64MultiArray>("vels_z", 1, true);
    acc_x_pub = nh.advertise<std_msgs::Float64MultiArray>("accs_x", 1, true);
    acc_y_pub = nh.advertise<std_msgs::Float64MultiArray>("accs_y", 1, true);
    acc_z_pub = nh.advertise<std_msgs::Float64MultiArray>("accs_z", 1, true);
    marker = visualization_msgs :: Marker();    

    std::mt19937 gen(3); // seed the generator
    std::uniform_int_distribution<> distr(5, 255); // define the range

    world_dimension.resize(3);
    world_dimension[0].resize(2);
    world_dimension[1].resize(2);
    world_dimension[2].resize(2);

        
    nh.getParam("/swarm_am_nav/world/resolution", world_resolution);
    nh.getParam("/swarm_am_nav/world/file_name", world_file_name);
    nh.getParam("/swarm_am_nav/rviz/display_sim", display_sim);
    nh.getParam("/swarm_am_nav/rviz/display_freq", display_freq);

    package_path = ros :: package::getPath("amswarmx");

    octomap::OcTree* octree_ptr = new octomap::OcTree(world_resolution);
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    if(!octree_ptr->readBinary(package_path + "/world/" + world_file_name)){
        throw std::invalid_argument("[AMSwarm] Fail to read octomap file.");
    }
    else{
        ROS_INFO_STREAM("Octomap read success! " << world_resolution);
    }

    octree_ptr->getMetricMin(world_dimension[0][0], world_dimension[1][0], world_dimension[2][0]);
    octree_ptr->getMetricMax(world_dimension[0][1], world_dimension[1][1], world_dimension[2][1]);

    if(world_file_name.compare("office.bt") == 0){
        x_max = +4.25;
        y_max = +6.5;
        x_min = -4.25;
        y_min = -6.5;
        z_max = 3.0;
        z_min = 0.0;
    }
    else{
        x_max = world_dimension[0][1];
        x_min = world_dimension[0][0];
        y_max = world_dimension[1][1];
        y_min = world_dimension[1][0];
        z_max = world_dimension[2][1];
        z_min = 0;//world_dimension[2][0];
    }
    distmap_obj = std::make_shared<DynamicEDTOctomap>(1.0, octree_ptr,
                                                          octomap::point3d(x_min, y_min, z_min), 
                                                          octomap::point3d(x_max, y_max, z_max),
                                                          false);
    distmap_obj->update();
    ROS_INFO_STREAM(x_max-x_min << " " << y_max-y_min << " " << z_max-z_min);
    params = YAML :: LoadFile(package_path+"/params/config_am_swarm.yaml");
    VERBOSE = params["verbose"].as<int>();
    
    num = params["num"].as<int>();
    num_drone = params["num_drone"].as<int>();

    agent_paths.markers.resize(num_drone);
    agent_convexpolys.markers.resize(1);
    agent_nonconvexsfcs.markers.resize(1);

    agent_boundaries.markers.resize(6);
    agent_griddatas.markers.resize(1);
    agent_vels_x.data.resize(1);
    agent_vels_y.data.resize(1);
    agent_vels_z.data.resize(1);

    agent_accs_x.data.resize(1);
    agent_accs_y.data.resize(1);
    agent_accs_z.data.resize(1);

    world = params["world"].as<int>();
    cluster = params["cluster"].as<int>();
    visualize_am = params["visualize_am"].as<bool>();
    trials = params["trials"].as<int>();
    write_files = params["write_files"].as<bool>();
    
    sfcc = params["sfcc"].as<bool>();

    camera_follow_drone = params["camera_follow_drone"].as<int>();
    free_space = params["free_space"].as<bool>();

    a_drone = params["a_drone"].as<double>();
    b_drone = params["b_drone"].as<double>();
    c_drone = params["c_drone"].as<double>();
    buffer = params["buffer"].as<double>();

    random_config = params["random_config"].as<bool>();
    pieces = params["pieces"].as<int>();
    degree = params["degree"].as<int>();

    max_iter = params["max_iter"].as<int>();
    max_time = params["max_time"].as<double>();
    t_plan = params["t_plan"].as<double>();
    dist_stop = params["dist_stop"].as<double>();
    dt = t_plan/num;

    success = false;
    collision_agent = false;
    collision_obstacle = false;
    saved = false;

    prob_data = new probData[num_drone];
    

    agents_x = Eigen :: ArrayXXd(num_drone, num); 
    agents_y = Eigen :: ArrayXXd(num_drone, num); 
    agents_z = Eigen :: ArrayXXd(num_drone, num);

    anchor_x = Eigen :: ArrayXXd(num_drone, num); 
    anchor_y = Eigen :: ArrayXXd(num_drone, num); 
    anchor_z = Eigen :: ArrayXXd(num_drone, num);

    smoothness = Eigen :: ArrayXd(num_drone);
    arc_length = Eigen :: ArrayXd(num_drone);
    dist_to_goal = Eigen :: ArrayXd(num_drone);

    
    if(!random_config){
        _init_drone = params["init_drone"].as< std :: vector<std::vector<double>>>();
        _goal_drone = params["goal_drone"].as< std :: vector<std::vector<double>>>();
    }
    else{
        auto [temp_init_drone, temp_goal_drone] = generateInitialAndGoalPositions(num_drone, x_min+1.5*a_drone, x_max-1.5*a_drone, y_min+1.5*a_drone, y_max-1.5*a_drone, 0.5, 0.7, a_drone*3.0, distmap_obj);
        _init_drone = temp_init_drone;
        _goal_drone = temp_goal_drone;
    }

    _anchor_points = params["anchor_points"].as< std :: vector<std::vector<std :: vector<double>>>>();
    _pos_static_obs = params["pos_static_obs"].as< std :: vector<std::vector<double>>>();
    _dim_static_obs = params["dim_static_obs"].as< std :: vector<std::vector<double>>>();
    
    num_obs = _pos_static_obs.size();
    if(free_space) num_obs = 0;
    
    if(_pos_static_obs.size() != _dim_static_obs.size())
        ROS_ERROR_STREAM("pos_obs and dim_obs sizes do not match");

    if(num_drone > _init_drone.size()){
        num_drone = _init_drone.size();
        ROS_WARN_STREAM("num_drone does not match size with given start-goal configurations");
    }
    ROS_INFO_STREAM("Number of drones " << num_drone);
    
    for(int i = 0; i < num_drone; i++){
        prob_data[i].id_badge = i;
        prob_data[i].num_drone = num_drone - 1;
        prob_data[i].x_init = _init_drone[i][0];
        prob_data[i].y_init = _init_drone[i][1];
        prob_data[i].z_init = _init_drone[i][2];

        prob_data[i].x_goal = _goal_drone[i][0];
        prob_data[i].y_goal = _goal_drone[i][1];
        prob_data[i].z_goal = _goal_drone[i][2];

        prob_data[i].xf_goal = _goal_drone[i][0];
        prob_data[i].yf_goal = _goal_drone[i][1];
        prob_data[i].zf_goal = _goal_drone[i][2];
        
        prob_data[i].anchor_points = _anchor_points[i];
        prob_data[i].pos_static_obs = _pos_static_obs;
        prob_data[i].dim_static_obs = _dim_static_obs;
        
        prob_data[i].params = params;
        prob_data[i].mpc_step = 0;
        if(visualize_am)
            prob_data[i].max_iter = 0;
        else
            prob_data[i].max_iter = max_iter;
        
        prob_data[i].octree_ptr = octree_ptr;
        prob_data[i].distmap_obj = distmap_obj;

        prob_data[i].world_resolution = world_resolution;
        prob_data[i].world_dimension = world_dimension;
        colors.push_back({distr(gen)/255.0, distr(gen)/255.0, distr(gen)/255.0});
    }
    
}

void AMSwarmX :: writeLAUNCH(){
    std::ofstream launchFile;
    launchFile.open(package_path+"/data/" + json_name + ".launch");

    if (launchFile.is_open()) {
        launchFile << "<launch>\n\n";
        launchFile << "  <arg name=\"world_frame_id\" default=\"world\"/>\n";
        launchFile << "  <arg name=\"world_file_name\" default=\"" << world_file_name << "\"/>\n";
        launchFile << "  <arg name=\"world_resolution\" default=\"" << world_resolution << "\"/>\n";
        launchFile << "  <node pkg=\"octomap_server\" type=\"octomap_server_node\" name=\"octomap_server\"\n";
        launchFile << "      args=\"$(find plan_env)/world/$(arg world_file_name)\">\n";
        launchFile << "    <param name=\"resolution\"                 value=\"$(arg world_resolution)\" />\n";
        launchFile << "    <param name=\"frame_id\"                   value=\"$(arg world_frame_id)\" type=\"string\"/>\n";
        launchFile << "    <param name=\"sensor_model/max_range\"     value=\"50.0\" />\n";
        launchFile << "    <param name=\"height_map\"                 value=\"false\"/>\n";
        launchFile << "    <param name=\"color/r\"                    value=\"0.2\" />\n";
        launchFile << "    <param name=\"color/g\"                    value=\"0.2\" />\n";
        launchFile << "    <param name=\"color/b\"                    value=\"0.2\" />\n";
        launchFile << "    <param name=\"color/a\"                    value=\"0.2\" />\n";
        launchFile << "  </node>\n\n";

        launchFile << "  <arg name=\"map_size_x\" value=\"" << x_max-x_min << "\"/>\n"; 
        launchFile << "  <arg name=\"map_size_y\" value=\"" << y_max-y_min << "\"/>\n";
        launchFile << "  <arg name=\"map_size_z\" value=\"" << z_max-z_min << "\"/>\n";
        launchFile << "  <arg name=\"odom_topic\" value=\"visual_slam/odom\" />\n";
       
        
        // Customizable values for init_x, init_y, init_z, target_x, target_y, target_z
        for(int i = 0; i < num_drone; i++){
            launchFile << "  <include file=\"$(find ego_planner)/launch/run_in_sim.launch\">\n";
            launchFile << "    <arg name=\"drone_id\" value=\"" << i << "\"/>\n";
            launchFile << "    <arg name=\"init_x\" value=\"" << _init_drone[i][0] << "\"/>\n";
            launchFile << "    <arg name=\"init_y\" value=\"" << _init_drone[i][1] << "\"/>\n";
            launchFile << "    <arg name=\"init_z\" value=\"" << _init_drone[i][2] << "\"/>\n\n";
            launchFile << "    <arg name=\"target_x\" value=\"" << _goal_drone[i][0] << "\"/>\n";
            launchFile << "    <arg name=\"target_y\" value=\"" << _goal_drone[i][1] << "\"/>\n";
            launchFile << "    <arg name=\"target_z\" value=\"" << _goal_drone[i][2] << "\"/>\n\n";
            
            launchFile << "    <arg name=\"map_size_x\" value=\"$(arg map_size_x)\"/>\n";
            launchFile << "    <arg name=\"map_size_y\" value=\"$(arg map_size_y)\"/>\n";
            launchFile << "    <arg name=\"map_size_z\" value=\"$(arg map_size_z)\"/>\n";
            launchFile << "    <arg name=\"odom_topic\" value=\"$(arg odom_topic)\"/>\n";
            launchFile << "  </include>\n";
        }
        launchFile << "</launch>\n";

        launchFile.close();
        std::cout << "Launch file created successfully!" << std::endl;
    } else {
        std::cout << "Failed to create launch file." << std::endl;
    }
} 

void AMSwarmX :: writeJSON(){

    double vel_max = params["vel_max"].as<double>()/sqrt(3);
    double gravity = params["gravity"].as<double>();
    double f_min = params["f_min"].as<double>() * gravity;
    double f_max = params["f_max"].as<double>() * gravity;
    double acc_max = (-(2*gravity) + sqrt(pow(2*gravity,2) - 4*3*(pow(gravity,2) - pow(f_max,2))))/6.0; 
    double z_acc_min = f_min - gravity;

    // Create the JSON root object
    Json::Value root;

    // Create the "quadrotors" object
    Json::Value quadrotors;
    Json::Value crazyflie;
    Json::Value default_quad;

    // Fill in the values for "crazyflie" quadrotor
    crazyflie["max_vel"].append(vel_max);
    crazyflie["max_vel"].append(vel_max);
    crazyflie["max_vel"].append(vel_max);
    crazyflie["max_acc"].append(acc_max);
    crazyflie["max_acc"].append(acc_max);
    crazyflie["max_acc"].append(z_acc_min);
    crazyflie["radius"] = a_drone;
    crazyflie["nominal_velocity"] = vel_max;
    crazyflie["downwash"] = c_drone/a_drone;

    // Fill in the values for "default" quadrotor
    default_quad["max_vel"].append(vel_max);
    default_quad["max_vel"].append(vel_max);
    default_quad["max_vel"].append(vel_max);
    default_quad["max_acc"].append(acc_max);
    default_quad["max_acc"].append(acc_max);
    default_quad["max_acc"].append(z_acc_min);
    default_quad["radius"] = a_drone;
    default_quad["nominal_velocity"] = vel_max;
    default_quad["downwash"] = c_drone/a_drone;

    // Add "crazyflie" and "default" objects to "quadrotors"
    quadrotors["crazyflie"] = crazyflie;
    quadrotors["default"] = default_quad;

    // Add "quadrotors" object to the root
    root["quadrotors"] = quadrotors;

    // Create the "world" array
    Json::Value world;
    Json::Value dimension;
    
    // Fill in the values for "dimension"
    dimension["dimension"].append(x_min+a_drone);
    dimension["dimension"].append(y_min+a_drone);
    dimension["dimension"].append(z_min+4*a_drone);
    dimension["dimension"].append(x_max-a_drone);
    dimension["dimension"].append(y_max-a_drone);
    dimension["dimension"].append(z_max-a_drone);
    
    // Add "dimension" to "world"
    world.append(dimension);
    // Add "world" array to the root
    root["world"] = world;

    // Create the "agents" array
    Json::Value agents;
    
    // Fill in the values for "agents"
    for (int i = 0; i < num_drone; i++) {
        Json::Value agent;
        agent["type"] = "crazyflie";
        agent["cid"] = i+1;
        agent["start"].append(_init_drone[i][0]);
        agent["start"].append(_init_drone[i][1]);
        agent["start"].append(_init_drone[i][2]);
        agent["goal"].append(_goal_drone[i][0]);
        agent["goal"].append(_goal_drone[i][1]);
        agent["goal"].append(_goal_drone[i][2]);
        agents.append(agent);
    }
    
    // Add "agents" array to the root
    root["agents"] = agents;

    // Create the "obstacles" array
    Json::Value obstacles(Json::arrayValue);
    
    // Add "obstacles" array to the root
    root["obstacles"] = obstacles;

    ROS_INFO_STREAM("Saving JSON file");
    // Save the JSON to a file
     // Save the JSON to a file without line breaks
    std::ofstream configFile(package_path+"/data/" + json_name + ".json");
    configFile << std::fixed << std::setprecision(4);
    configFile << root;
    configFile.close();
}

void AMSwarmX :: shareInformation(){
    // Update common variable
    for(int i = 0; i < num_drone; i++){
        if(sim_iter == 0){
            agents_x.row(i) = Eigen :: ArrayXXd :: Ones(1, num) * _init_drone[i][0];
            agents_y.row(i) = Eigen :: ArrayXXd :: Ones(1, num) * _init_drone[i][1];
            agents_z.row(i) = Eigen :: ArrayXXd :: Ones(1, num) * _init_drone[i][2];

            anchor_x.row(i) = Eigen :: ArrayXXd :: Ones(1, num) * _init_drone[i][0];
            anchor_y.row(i) = Eigen :: ArrayXXd :: Ones(1, num) * _init_drone[i][1];
            anchor_z.row(i) = Eigen :: ArrayXXd :: Ones(1, num) * _init_drone[i][2];
        }
        else{
            agents_x.row(i) = prob_data[i].x.transpose();
            agents_y.row(i) = prob_data[i].y.transpose();
            agents_z.row(i) = prob_data[i].z.transpose();

            anchor_x.row(i) = prob_data[i].x_anchor.transpose();
            anchor_y.row(i) = prob_data[i].y_anchor.transpose();
            anchor_z.row(i) = prob_data[i].z_anchor.transpose();
        }
    }

    // Share information
    for(int i = 0; i < num_drone; i++){
        prob_data[i].agents_x = Eigen :: ArrayXXd(num_drone, num);
        prob_data[i].agents_y = Eigen :: ArrayXXd(num_drone, num);
        prob_data[i].agents_z = Eigen :: ArrayXXd(num_drone, num);

        prob_data[i].agents_x = agents_x;
        prob_data[i].agents_y = agents_y;
        prob_data[i].agents_z = agents_z;
    }
}

void AMSwarmX :: checkCollision(){
    std :: vector <double> temp_agent_obs;
    // Collision check
    for(int i = 0; i < num_drone; i++){
        Eigen :: ArrayXd coll;
        coll = pow((agents_x(i, 0) - agents_x.col(0))/(2*a_drone), 2) 
            + pow((agents_y(i, 0) - agents_y.col(0))/(2*b_drone), 2)
            + pow((agents_z(i, 0) - agents_z.col(0))/(2*c_drone), 2); 
        for(int j = 0; j < num_drone; j++){
            if(i!=j){
                double val = pow(agents_x(i, 0) - agents_x(j, 0), 2)/pow(2*a_drone, 2)
                        + pow(agents_y(i, 0) - agents_y(j, 0), 2)/pow(2*b_drone, 2)
                        + pow(agents_z(i, 0) - agents_z(j, 0), 2)/pow(2*c_drone, 2);
                if(val < 1.0){    
                    ROS_WARN_STREAM("Collision between agent " << i << " and agent " << j << " at MPC step " << prob_data[i].mpc_step);   
                    collision_agent = true;
                }
            }
        }
        
        
        float dist_obs;
        octomap::point3d obstacle_pos;
        octomap::point3d agent_pos(prob_data[i].x_init, prob_data[i].y_init, prob_data[i].z_init);
        prob_data[i].distmap_obj->getDistanceAndClosestObstacle(agent_pos, dist_obs, obstacle_pos);
        // double dist_obs2 = prob_data[i].distmap_obj->getDistance(agent_pos);
        temp_agent_obs.push_back(dist_obs);
        if(dist_obs < a_drone + 0*prob_data[i].world_resolution/2.0){
            ROS_WARN_STREAM("Collision between agent " << i << agent_pos
                            << " and static obstacle " << obstacle_pos << " at MPC step " << prob_data[i].mpc_step);   
            collision_obstacle = true;
        }
    }
    if(temp_agent_obs.size() != 0)
        agent_obs_dist.push_back(*std::min_element(temp_agent_obs.begin(), temp_agent_obs.end()));

}

void AMSwarmX :: checkAtGoal(){
    // Dist-to-goal check
    for(int i = 0; i < num_drone; i++){
        dist_to_goal(i) = prob_data[i].dist_to_goal;
    }
    if((dist_to_goal < dist_stop).all()){
        success = true; 
    }
}

void AMSwarmX :: solveCluster(int start_index){
    for(int i = start_index; i < cluster+start_index && i < num_drone; i++){
         deployAgent(prob_data[i], VERBOSE);
    }
}
void AMSwarmX :: runAlgorithm(){

    static std :: vector <std :: thread> cluster_threads;

    // Start Threads
    ros::WallTime start, end;
    start = ros::WallTime::now();
    
    for(int i = 0; i < num_drone; i+=cluster){
       cluster_threads.emplace_back(std::bind(&AMSwarmX::solveCluster, this, i));
    }
    // Waiting for threads to terminate
    for(int i = 0; i < cluster_threads.size(); i++)
        cluster_threads[i].join();
    

    end = ros::WallTime::now();
    double total_time = (end - start).toSec();
    comp_time_agent.push_back(total_time/num_drone);//total_time.count()/1000.0/num_drone);

    if(VERBOSE == 4)
        ROS_INFO_STREAM("Time to compute = " << total_time << " s, Planning Frequency = " << 1.0/total_time);
    
    cluster_threads.clear();
}

void AMSwarmX :: applyControl(){
    
    for(int i = 0; i < num_drone; i++){
        prob_data[i].mpc_step++;

        // @ Apply control input				
        prob_data[i].ax_init += prob_data[i].xdddot(1) * prob_data[i].dt;
        prob_data[i].ay_init += prob_data[i].ydddot(1) * prob_data[i].dt;
        prob_data[i].az_init += prob_data[i].zdddot(1) * prob_data[i].dt;

        // double acc_alpha = atan2(prob_data[i].ay_init, prob_data[i].ax_init);
        // double acc_beta = (prob_data[i].az_init == 0.0) ? M_PI_2 : atan2((prob_data[i].ax_init/cos(acc_alpha)), prob_data[i].az_init + prob_data[i].gravity);
        // double acc_cmd = sqrt(pow(prob_data[i].ax_init, 2) + pow(prob_data[i].ay_init, 2) + pow(prob_data[i].az_init + prob_data[i].gravity, 2)); 
        // double saturated_acc_cmd = std :: min(acc_cmd, prob_data[i].f_max);
      
        // prob_data[i].ax_init = saturated_acc_cmd * cos(acc_alpha) * sin(acc_beta);
        // prob_data[i].ay_init = saturated_acc_cmd * sin(acc_alpha) * sin(acc_beta);
        // prob_data[i].az_init = -prob_data[i].gravity + saturated_acc_cmd * cos(acc_beta);

        prob_data[i].vx_init += prob_data[i].ax_init * prob_data[i].dt + 0.5 * prob_data[i].xdddot(1) * pow(prob_data[i].dt, 2);
        prob_data[i].vy_init += prob_data[i].ay_init * prob_data[i].dt + 0.5 * prob_data[i].ydddot(1) * pow(prob_data[i].dt, 2);
        prob_data[i].vz_init += prob_data[i].az_init * prob_data[i].dt + 0.5 * prob_data[i].zdddot(1) * pow(prob_data[i].dt, 2);

        // double vel_alpha = atan2(prob_data[i].vy_init, prob_data[i].vx_init);
        // double vel_beta = (prob_data[i].vz_init == 0.0) ? M_PI_2 : atan2((prob_data[i].vx_init/cos(vel_alpha)), prob_data[i].vz_init);
        // double vel_cmd = sqrt(pow(prob_data[i].vx_init, 2) + pow(prob_data[i].vy_init, 2) + pow(prob_data[i].vz_init, 2));
        // double saturated_vel_cmd = std :: min(vel_cmd, prob_data[i].vel_max);

        // prob_data[i].vx_init = saturated_vel_cmd * cos(vel_alpha) * sin(vel_beta);
        // prob_data[i].vy_init = saturated_vel_cmd * sin(vel_alpha) * sin(vel_beta);
        // prob_data[i].vz_init = saturated_vel_cmd * cos(vel_beta);

        prob_data[i].x_init += prob_data[i].vx_init * prob_data[i].dt + 0.5 * prob_data[i].ax_init * pow(prob_data[i].dt, 2) + (1.0/6.0) * prob_data[i].xdddot(1) * pow(prob_data[i].dt, 3);
        prob_data[i].y_init += prob_data[i].vy_init * prob_data[i].dt + 0.5 * prob_data[i].ay_init * pow(prob_data[i].dt, 2) + (1.0/6.0) * prob_data[i].ydddot(1) * pow(prob_data[i].dt, 3) ;
        prob_data[i].z_init += prob_data[i].vz_init * prob_data[i].dt + 0.5 * prob_data[i].az_init * pow(prob_data[i].dt, 2) + (1.0/6.0) * prob_data[i].zdddot(1) * pow(prob_data[i].dt, 3) ;
    }
}

void AMSwarmX :: checkViolation(){
    // check for any violations in pos, vel, acc bounds
    for(int i = 0; i < num_drone; i++){
        // check for any violations in pos, vel, acc bounds
		
        if(std::floor(prob_data[i].x_init/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].x_max || std::ceil(prob_data[i].x_init/prob_data[i].thresold)*prob_data[i].thresold < prob_data[i].x_min){
			out_space = true;
            ROS_WARN_STREAM("Positional bounds not satisfied in x " << prob_data[i].x_init << " " << prob_data[i].x_min << " " << prob_data[i].x_max);
        }
		if(std::floor(prob_data[i].y_init/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].y_max  || std::ceil(prob_data[i].y_init/prob_data[i].thresold)*prob_data[i].thresold < prob_data[i].y_min){
			out_space = true;
            ROS_WARN_STREAM("Positional bounds not satisfied in y " << prob_data[i].y_init << " " << prob_data[i].y_min << " " << prob_data[i].y_max);
        }
		if(std::floor(prob_data[i].z_init/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].z_max  || std::ceil(prob_data[i].z_init/prob_data[i].thresold)*prob_data[i].thresold < prob_data[i].z_min){
			out_space = true;
            ROS_WARN_STREAM("Positional bounds not satisfied in z " << prob_data[i].z_init << " " << prob_data[i].z_min << " " << prob_data[i].z_max);
        }
		if(prob_data[i].axis_wise){
			if(std::floor(abs(prob_data[i].vx_init)/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].vel_max + 0.01){
				ROS_WARN_STREAM("Velocity bounds not satisfied in x " << abs(prob_data[i].vx_init) << " " << prob_data[i].vel_max);
                out_space = true;
            }
			if(std::floor(abs(prob_data[i].vy_init)/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].vel_max + 0.01){
				ROS_WARN_STREAM("Velocity bounds not satisfied in y " << abs(prob_data[i].vy_init) << " " << prob_data[i].vel_max);
                out_space = true;
            }
			if(std::floor(abs(prob_data[i].vz_init)/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].vel_max + 0.01){
				ROS_WARN_STREAM("Velocity bounds not satisfied in z " << abs(prob_data[i].vz_init) << " " << prob_data[i].vel_max);
                out_space = true;    
            }
			double acc_control = sqrt(pow(prob_data[i].ax_init,2) + pow(prob_data[i].ay_init,2) + pow(prob_data[i].gravity+prob_data[i].az_init,2));
			if(std::floor(acc_control/prob_data[i].thresold/10)*prob_data[i].thresold*10 > prob_data[i].f_max + 0.01 || std::ceil(acc_control/prob_data[i].thresold/10)*prob_data[i].thresold*10 < prob_data[i].f_min - 0.01){
                ROS_WARN_STREAM("Acceleration bounds not satisfied " << acc_control << " " << prob_data[i].f_min << " " << prob_data[i].f_max);
                out_space = true;
            }

		}
		else{
			double acc_control = sqrt(pow(prob_data[i].ax_init,2) + pow(prob_data[i].ay_init,2) + pow(prob_data[i].gravity+prob_data[i].az_init,2));
			double vel_control = sqrt(pow(prob_data[i].vx_init,2) + pow(prob_data[i].vy_init,2) + pow(prob_data[i].vz_init,2));
			if(std::floor(vel_control/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].vel_max + 0.01 ){
                out_space = true;
				ROS_WARN_STREAM("Velocity bounds not satisfied " << vel_control << " " << prob_data[i].vel_max);
			}
			if(std::floor(acc_control/prob_data[i].thresold/10)*prob_data[i].thresold*10 > prob_data[i].f_max + 0.01  || std::ceil(acc_control/prob_data[i].thresold/10)*prob_data[i].thresold*10 < prob_data[i].f_min - 0.01){
                out_space = true;
				ROS_WARN_STREAM("Acceleration bounds not satisfied " << acc_control << " " << prob_data[i].f_min << " " << prob_data[i].f_max);
            }
		}
    }
}

void AMSwarmX :: runSimulation(){
    
  
    // save_data.open(package_path+"/data/" + name + "_sim_data.txt");
    save_data_2.open(package_path+"/data/" + result_name + "_sim_residue.txt");

    auto start = std :: chrono :: high_resolution_clock::now();            
    ros::Rate rate(display_freq);

    sim_iter = 0;
    while(ros::ok()){
        
        if(sim_iter < max_time/dt && !success){
        
            out_space = false;
            
            AMSwarmX :: shareInformation();
            // if(display_sim)
            AMSwarmX :: publishDataAlgorithm();
           
            AMSwarmX :: runAlgorithm();
            AMSwarmX :: applyControl();
            AMSwarmX :: checkCollision();
            AMSwarmX :: checkViolation();
            AMSwarmX :: checkAtGoal();
            AMSwarmX :: calculateDistances();
           
            mission_time = (sim_iter+1)*dt; 
            // save_data << agents_x << "\n" << agents_y << "\n" << agents_z << "\n"
            //         << anchor_x << "\n" << anchor_y << "\n" << anchor_z << "\n";
                        
            // for(int i = 0; i < num_drone; i++){
            //     save_data_2 << prob_data[i].x_goal << " " << prob_data[i].y_goal << " " << prob_data[i].z_goal << "\n";
            // }
            for(int i = 0; i < num_drone; i++){
                save_data_2 << prob_data[i].res_x_sfc_norm << " " << prob_data[i].res_y_sfc_norm << " " << prob_data[i].res_z_sfc_norm 
                            << " " << prob_data[i].res_x_drone_norm << " " << prob_data[i].res_y_drone_norm << " " << prob_data[i].res_z_drone_norm
                            << " " << prob_data[i].res_x_vel_norm << " " << prob_data[i].res_y_vel_norm << " " << prob_data[i].res_z_vel_norm
                            << " " << prob_data[i].res_x_acc_norm << " " << prob_data[i].res_y_acc_norm << " " << prob_data[i].res_z_acc_norm
                            << " " << prob_data[i].res_x_ineq_norm << " " << prob_data[i].res_y_ineq_norm << " " << prob_data[i].res_z_ineq_norm << "\n";
            }


            if(VERBOSE == 2){
                ROS_INFO_STREAM("Simulation time = " << sim_iter);
                ROS_INFO_STREAM("Distance to goals = " << dist_to_goal.transpose());
            }
            sim_iter++;

            if(collision_agent || collision_obstacle)
                break;
        }
        else{
            if(!saved){
                auto end = std :: chrono :: high_resolution_clock::now();
                total_time = end - start;
                // save_data.close();
                save_data_2.close();
                saved = true;
                AMSwarmX :: saveMetrics();
                break;
            }
        }
        rate.sleep();
    }
    // ros::Duration(1.5).sleep();
}

void AMSwarmX :: publishDataAlgorithm(){
    int temp_id = 0;

    agent_trajs.markers.clear(); 
    agent_startgoals.markers.clear(); 
    agent_anchors.markers.clear();
    agent_griddatas.markers.clear(); 
    agent_gridpaths.markers.clear();
    agent_nonconvexsfcs.markers.clear(); 

    agent_paths.markers.resize(num_drone);
    agent_convexpolys.markers.resize(1);
    agent_nonconvexsfcs.markers.resize(1);

    agent_boundaries.markers.resize(6);
    agent_griddatas.markers.resize(1);
    agent_vels_x.data.resize(1);
    agent_vels_y.data.resize(1);
    agent_vels_z.data.resize(1);

    agent_accs_x.data.resize(1);
    agent_accs_y.data.resize(1);
    agent_accs_z.data.resize(1);

    for(int i = 0; i < num_drone; i++){

        if(i==camera_follow_drone && sim_iter!=0){
            double yaw = atan2(prob_data[i].y(1)-prob_data[i].y(0), prob_data[i].x(1)-prob_data[i].x(0));
			double pitch = atan2((prob_data[i].x(1) - prob_data[i].x(0))/cos(yaw), prob_data[i].z(1) - prob_data[i].z(0));

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(prob_data[i].x_init, prob_data[i].y_init, prob_data[i].z_init) );
            tf::Quaternion q;
            
            q.setRPY(0, 0, M_PI+yaw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "fps"));
        } 

        // crazyflie dae
        marker = visualization_msgs :: Marker();
        marker.header.frame_id = "/map";
        marker.ns = "Crazyflie";
        marker.id = temp_id++;
        marker.action = marker.ADD;
        marker.type = marker.MESH_RESOURCE;
        marker.mesh_use_embedded_materials = true;
        if(sim_iter == 0)
            marker.lifetime = ros::Duration(dt);
        marker.mesh_resource = "package://amswarmx/crazyflie/meshes/crazyflie.dae";
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        
        Eigen :: Vector3f thrust;
        thrust << prob_data[i].ax_init, prob_data[i].ay_init, prob_data[i].az_init + prob_data[i].gravity;
        // auto euler = computeQuaternions(thrust);
        // tf::Quaternion q_;
        // q_.setRPY(euler(0), euler(1), euler(2));
        marker.pose.orientation.x = 0;//q_.getX();//0.0;
        marker.pose.orientation.y = 0;//q_.getY();//0.0;
        marker.pose.orientation.z = 0;//q_.getZ();//0.0;
        marker.pose.orientation.w = 1;//q_.getW();//1.0;  
        marker.pose.position.x = agents_x(i, 0) - 0.015;
        marker.pose.position.y = agents_y(i, 0); 
        marker.pose.position.z = agents_z(i, 0); 
        agent_trajs.markers.push_back(marker);  
       

        // ellipse and trajectory
        marker = visualization_msgs :: Marker();
        marker.header.frame_id = "/map";
        marker.ns = "Ellipse";
        marker.id = temp_id++;
        marker.type = marker.SPHERE;
        marker.action = marker.ADD;
        marker.scale.x = 2*(a_drone) + 0*buffer;
        marker.scale.y = 2*(b_drone) + 0*buffer;
        marker.scale.z = 2*(c_drone) + 0*buffer;
        if(sim_iter == 0)
            marker.lifetime = ros::Duration(dt);
        marker.color.a = 0.5; 
        marker.color.r = colors[i][0];
        marker.color.g = colors[i][1];
        marker.color.b = colors[i][2];
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = agents_x(i, 0);
        marker.pose.position.y = agents_y(i, 0); 
        marker.pose.position.z = agents_z(i, 0); 
        agent_trajs.markers.push_back(marker); 
        

        // sphere
        marker = visualization_msgs :: Marker();
        marker.header.frame_id = "/map";
        marker.ns = "Sphere";
        marker.id = temp_id++;
        marker.type = marker.SPHERE;
        marker.action = marker.ADD;
        marker.scale.x = 2*(a_drone) + 0*buffer;
        marker.scale.y = 2*(b_drone) + 0*buffer;
        marker.scale.z = 2*(a_drone) + 0*buffer;
        if(sim_iter == 0)
            marker.lifetime = ros::Duration(dt);
        marker.color.a = 0.5; 
        marker.color.r = colors[i][0];
        marker.color.g = colors[i][1];
        marker.color.b = colors[i][2];
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = agents_x(i, 0);
        marker.pose.position.y = agents_y(i, 0); 
        marker.pose.position.z = agents_z(i, 0); 
        agent_trajs.markers.push_back(marker);  
       

        // if(i == 0){
            marker = visualization_msgs :: Marker();
            for(int j = 0; j < num; j++){
                marker.header.frame_id = "/map";
                marker.ns = "Trajectory";
                marker.id = temp_id++;
                marker.type = marker.LINE_STRIP;
                marker.action = marker.ADD;
                marker.scale.x = (a_drone) + 0*buffer/2.0;
                marker.color.a = 0.5; 
                marker.color.r = colors[i][0];
                marker.color.g = colors[i][1];
                marker.color.b = colors[i][2];
                // marker.lifetime = ros::Duration(dt);
                marker.pose.orientation.w = 1.0;
                marker.points.push_back(buildPoint(agents_x(i, j), agents_y(i, j), agents_z(i, j)));
            }
            agent_trajs.markers.push_back(marker);  
        // }
        // start
        // marker = visualization_msgs :: Marker();
        // marker.header.frame_id = "/map";
        // marker.ns = "Start"+std::to_string(i);
        // marker.id = temp_id++;
        // marker.type = marker.SPHERE;
        // marker.action = marker.ADD;
        // marker.scale.x = 0.1;
        // marker.scale.y = 0.1;
        // marker.scale.z = 0.1;
        // marker.color.a = 1.0;
        // marker.color.r = colors[i][0];
        // marker.color.g = colors[i][1];
        // marker.color.b = colors[i][2];
        // marker.pose.orientation.w = 1.0;
        // marker.pose.position.x = _init_drone[i][0];
        // marker.pose.position.y = _init_drone[i][1]; 
        // marker.pose.position.z = _init_drone[i][2];
        // marker.text = "S"+std::to_string(i);
        // agent_startgoals.markers.push_back(marker);

        // goal                
        marker = visualization_msgs :: Marker();
        marker.header.frame_id = "/map";
        marker.ns = "Goal"+std::to_string(i);
        marker.id = temp_id++;
        marker.type = marker.CUBE;
        marker.action = marker.ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        //marker.lifetime = ros::Duration(dt);
        marker.color.a = 1.0;
        marker.color.r = colors[i][0];
        marker.color.g = colors[i][1];
        marker.color.b = colors[i][2];
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = _goal_drone[i][0];
        marker.pose.position.y = _goal_drone[i][1]; 
        marker.pose.position.z = _goal_drone[i][2];
        // marker.text = "G"+std::to_string(i);
        agent_startgoals.markers.push_back(marker);

        if(i == 0){
            // anchors
            for(int j = 0; j < pieces; j++){
                marker = visualization_msgs :: Marker();
                marker.header.frame_id = "/map";
                marker.id = temp_id++;
                marker.type = marker.CUBE;
                marker.action = marker.ADD;
                marker.ns = "Anchor"+std::to_string(i);
                marker.scale.x = 0.1;//2*a_drone + buffer;
                marker.scale.y = 0.1;//2*a_drone + buffer;
                marker.scale.z = 0.1;//2*c_drone + buffer;
                marker.color.a = 1.0;
                marker.color.r = colors[i][0];
                marker.color.g = colors[i][1];
                marker.color.b = colors[i][2];
                marker.pose.orientation.w = 1.0;
                //marker.lifetime = ros::Duration(dt);
                marker.pose.position.x = anchor_x(i, j*num/pieces);
                marker.pose.position.y = anchor_y(i, j*num/pieces); 
                marker.pose.position.z = anchor_z(i, j*num/pieces);  
                agent_anchors.markers.push_back(marker);
            }
            if(sim_iter!=0){
                marker = visualization_msgs :: Marker();
                marker.header.frame_id = "/map";
                marker.id = temp_id++;
                marker.type = marker.SPHERE;
                marker.action = marker.ADD;
                marker.ns = "InterGoal"+std::to_string(i);
                marker.scale.x = 0.1; //2*a_drone + buffer;
                marker.scale.y = 0.1; //2*a_drone + buffer;
                marker.scale.z = 0.1; //2*c_drone + buffer;
                marker.color.a = 1.0;
                marker.color.r = colors[i][0];
                marker.color.g = colors[i][1];;//;//colors[i][1];
                marker.color.b = colors[i][2];;//colors[i][2];
                //marker.lifetime = ros::Duration(dt);
                marker.pose.orientation.w = 1.0;
                marker.pose.position.x = prob_data[i].x_goal;
                marker.pose.position.y = prob_data[i].y_goal; 
                marker.pose.position.z = prob_data[i].z_goal;
                agent_anchors.markers.push_back(marker);
            }
        }
        
        // path
        agent_paths.markers[i].header.frame_id = "/map";
        agent_paths.markers[i].ns = "Path"+std::to_string(i);
        agent_paths.markers[i].id = temp_id++;
        agent_paths.markers[i].type = marker.LINE_STRIP;
        agent_paths.markers[i].action = marker.ADD;
        agent_paths.markers[i].scale.x = 0.02;
        agent_paths.markers[i].color.a = 1.0;
        // marker.lifetime = ros::Duration(dt);
        agent_paths.markers[i].color.r = colors[i][0];
        agent_paths.markers[i].color.g = colors[i][1];
        agent_paths.markers[i].color.b = colors[i][2];
        agent_paths.markers[i].pose.orientation.w = 1.0;
        agent_paths.markers[i].points.push_back(buildPoint(agents_x(i, 0), agents_y(i, 0), agents_z(i, 0)));
        if(sim_iter == 0)
            agent_paths.markers[i].points.push_back(buildPoint(agents_x(i, 0), agents_y(i, 0), agents_z(i, 0)));     

        if(i == 0 && !prob_data[i].sfcc){
            agent_nonconvexsfcs.markers[i].header.frame_id = "/map";
            agent_nonconvexsfcs.markers[i].ns = "SFC"+std::to_string(i);
            agent_nonconvexsfcs.markers[i].id = temp_id++;
            agent_nonconvexsfcs.markers[i].type = marker.LINE_STRIP;
            agent_nonconvexsfcs.markers[i].action = marker.ADD;
            agent_nonconvexsfcs.markers[i].scale.x = 0.05;//(2*a_drone+buffer)/2;
            agent_nonconvexsfcs.markers[i].color.a = 1.0;
            // agent_nonconvexsfcs.markers[i].lifetime = ros::Duration(dt);
            agent_nonconvexsfcs.markers[i].color.r = colors[i][0];
            agent_nonconvexsfcs.markers[i].color.g = colors[i][1];
            agent_nonconvexsfcs.markers[i].color.b = colors[i][2];
            agent_nonconvexsfcs.markers[i].pose.orientation.w = 1.0;
           
            
            // for(int j = 0; j < prob_data[i].alpha_sfc.rows(); j++){
            //     double k = prob_data[i].alpha_sfc(j);
            //     octomap::point3d origin(prob_data[i].x_anchor(0), prob_data[i].y_anchor(0), prob_data[i].z_anchor(0));
            //     octomap::point3d direction(cos(k), sin(k), 0.0);
            //     octomap::point3d ray_end;    
            //     double d_sfc;
            //     castRobot(prob_data[i], origin, direction, ray_end, d_sfc);

            //     ray_end = origin + direction * (d_sfc + prob_data[i].distance_to_obs_margin);
            //     agent_nonconvexsfcs.markers[i].points.push_back(buildPoint(ray_end.x(), ray_end.y(), ray_end.z()));
            // }

            for(double k = 0; k <= 360.0; k=k+1){
                octomap::point3d origin(prob_data[i].x_anchor(0), prob_data[i].y_anchor(0), prob_data[i].z_anchor(0));
                octomap::point3d direction(cos(k*M_PI/180.0), sin(k*M_PI/180.0), 0.0);
                octomap::point3d ray_end;    
                double d_sfc;
                castRobot(prob_data[i], origin, direction, ray_end, d_sfc);

                ray_end = origin + direction * (d_sfc + prob_data[i].distance_to_obs_margin);
                agent_nonconvexsfcs.markers[i].points.push_back(buildPoint(ray_end.x(), ray_end.y(), ray_end.z()));
            }
        }
        else if(i == 0 && prob_data[i].sfcc){
            if(world == 2){
                auto ells = prob_data[0].decomp_util2.get_ellipsoids();
                // for(int i = 1; i < 0*num_drone; i++){
                //     auto agent_ells = prob_data[i].decomp_util2.get_ellipsoids();
                //     for(auto temp_ells:agent_ells)
                //         ells.push_back(temp_ells);
                // }
                decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(ells);
                es_msg.header.frame_id = "/map";
                es_pub.publish(es_msg);
                
                auto polys = prob_data[0].decomp_util2.get_polyhedrons();
                // for(int i = 1; i < 0*num_drone; i++){
                //     auto agent_polys = prob_data[i].decomp_util2.get_polyhedrons();
                //     for(auto temp_polys:agent_polys)
                //         polys.push_back(temp_polys);
                // }

                decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
                poly_msg.header.frame_id = "/map";
                convexpoly_pub.publish(poly_msg);
            }
            else{
                auto ells = prob_data[0].decomp_util.get_ellipsoids();
                // for(int i = 1; i < 0*num_drone; i++){
                //     auto agent_ells = prob_data[i].decomp_util.get_ellipsoids();
                //     for(auto temp_ells:agent_ells)
                //         ells.push_back(temp_ells);
                // }
                decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(ells);
                es_msg.header.frame_id = "/map";
                es_pub.publish(es_msg);
                
                auto polys = prob_data[0].decomp_util.get_polyhedrons();
                // for(int i = 1; i < 0*num_drone; i++){
                //     auto agent_polys = prob_data[i].decomp_util.get_polyhedrons();
                //     for(auto temp_polys:agent_polys)
                //         polys.push_back(temp_polys);
                // }

                decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
                poly_msg.header.frame_id = "/map";
                convexpoly_pub.publish(poly_msg);
            }
        }
        // gridpath
        for(int k = 0; k < prob_data[i].grid_path.size(); k++){
            marker = visualization_msgs :: Marker();
            marker.header.frame_id = "/map";
            marker.ns = "GridPath"+std::to_string(i);
            marker.id = temp_id++;
            marker.type = marker.SPHERE;
            marker.action = marker.ADD;
            marker.scale.x = 0.06;
            marker.scale.y = 0.06;
            marker.scale.z = 0.06;
            marker.color.a = 0.5;
            //marker.lifetime = ros::Duration(dt);
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = prob_data[i].grid_path[k][0];
            marker.pose.position.y = prob_data[i].grid_path[k][1]; 
            marker.pose.position.z = prob_data[i].grid_path[k][2];
            agent_gridpaths.markers.push_back(marker);
          
        }

        //griddata
        // if(i == 0 && sim_iter!=0){
        //     int temp_index = 0;
        //     for(int l = 0; l < prob_data[i].grid_z.rows(); l++){
        //         for(int j = 0; j < prob_data[i].grid_y.rows(); j++){
        //             for(int k = 0; k < prob_data[i].grid_x.rows(); k++){
        //                 marker = visualization_msgs :: Marker();
        //                 marker.header.frame_id = "/map";
        //                 marker.id = temp_id++;
        //                 marker.type = marker.SPHERE;
        //                 marker.action = marker.ADD;
        //                 if(prob_data[i].grid_data[temp_index]){
        //                     marker.color.a = 1.0;
        //                     marker.color.r = 1.0;
        //                     marker.color.g = 0.0;
        //                     marker.color.b = 0.0;                 
        //                     marker.scale.x = 0.02;
        //                     marker.scale.y = 0.02;
        //                     marker.scale.z = 0.02;                       
        //                     marker.pose.orientation.w = 1.0;
        //                     marker.pose.position.x = prob_data[i].grid_x(k);
        //                     marker.pose.position.y = prob_data[i].grid_y(j); 
        //                     marker.pose.position.z = prob_data[i].grid_z(l); 
        //                     agent_griddatas.markers.push_back(marker);
        //                     temp_index++;
        //                 }
        //                 else{
        //                     temp_index++;
        //                     continue;
        //                 }                  
        //             }
        //         }
        //     }
        // }

        // vels and accs
        if(i == 0){
            if(sim_iter == 0){
                agent_vels_x.data[i] = 0.0;
                agent_accs_x.data[i] = 0.0;
                agent_vels_y.data[i] = 0.0;
                agent_accs_y.data[i] = 0.0;
                agent_vels_z.data[i] = 0.0;
                agent_accs_z.data[i] = 0.0;
            }
            else{
                agent_vels_x.data[i] = prob_data[i].vx_init;
                agent_accs_x.data[i] = prob_data[i].ax_init;
                agent_vels_y.data[i] = prob_data[i].vy_init;
                agent_accs_y.data[i] = prob_data[i].ay_init;
                agent_vels_z.data[i] = prob_data[i].vz_init;
                agent_accs_z.data[i] = prob_data[i].az_init;
            }
        }
    }    
    
    for(int i = 0; i < agent_boundaries.markers.size(); i++){
        agent_boundaries.markers[i].points.clear();
        agent_boundaries.markers[i].header.frame_id = "/map";
        agent_boundaries.markers[i].id = temp_id++;
        agent_boundaries.markers[i].type = marker.LINE_STRIP;
        agent_boundaries.markers[i].action = marker.ADD;
        agent_boundaries.markers[i].scale.x = 0.05;
        agent_boundaries.markers[i].color.a = 1.0;
        agent_boundaries.markers[i].color.r = 0.0;
        agent_boundaries.markers[i].color.g = 0.0;
        agent_boundaries.markers[i].color.b = 0.0;
        agent_boundaries.markers[i].pose.orientation.w = 1.0;
    }

    agent_boundaries.markers[0].points.push_back(buildPoint(x_min, y_min, z_min));
    agent_boundaries.markers[0].points.push_back(buildPoint(x_min, y_max, z_min));
    agent_boundaries.markers[0].points.push_back(buildPoint(x_max, y_max, z_min));
    agent_boundaries.markers[0].points.push_back(buildPoint(x_max, y_min, z_min));
    agent_boundaries.markers[0].points.push_back(buildPoint(x_min, y_min, z_min));

    agent_boundaries.markers[1].points.push_back(buildPoint(x_min, y_min, z_max));
    agent_boundaries.markers[1].points.push_back(buildPoint(x_min, y_max, z_max));
    agent_boundaries.markers[1].points.push_back(buildPoint(x_max, y_max, z_max));
    agent_boundaries.markers[1].points.push_back(buildPoint(x_max, y_min, z_max));
    agent_boundaries.markers[1].points.push_back(buildPoint(x_min, y_min, z_max));
    
    agent_boundaries.markers[2].points.push_back(buildPoint(x_max, y_max, z_min));
    agent_boundaries.markers[2].points.push_back(buildPoint(x_max, y_max, z_max));

    agent_boundaries.markers[3].points.push_back(buildPoint(x_max, y_min, z_min));
    agent_boundaries.markers[3].points.push_back(buildPoint(x_max, y_min, z_max));

    agent_boundaries.markers[4].points.push_back(buildPoint(x_min, y_max, z_min));
    agent_boundaries.markers[4].points.push_back(buildPoint(x_min, y_max, z_max));

    agent_boundaries.markers[5].points.push_back(buildPoint(x_min, y_min, z_min));
    agent_boundaries.markers[5].points.push_back(buildPoint(x_min, y_min, z_max));
 
    traj_pub.publish(agent_trajs);
    startgoal_pub.publish(agent_startgoals);
    anchor_pub.publish(agent_anchors);
    path_pub.publish(agent_paths);
    if(sim_iter!=0){
        gridpath_pub.publish(agent_gridpaths);
        griddata_pub.publish(agent_griddatas);
        nonconvexsfc_pub.publish(agent_nonconvexsfcs);
    }
    vel_x_pub.publish(agent_vels_x);
    vel_y_pub.publish(agent_vels_y);
    vel_z_pub.publish(agent_vels_z);
    acc_x_pub.publish(agent_accs_x);
    acc_y_pub.publish(agent_accs_y);
    acc_z_pub.publish(agent_accs_z);
    boundary_pub.publish(agent_boundaries);
}


void AMSwarmX :: calculateDistances(){
    std :: vector <double> temp_inter_agent, temp_agent_obs;
    for(int i = 0; i < num_drone; i++){
        for(int j = i+1; j < num_drone; j++){
            temp_inter_agent.push_back(sqrt(pow(prob_data[i].x_init - prob_data[j].x_init, 2) 
                                            + pow(prob_data[i].y_init - prob_data[j].y_init, 2)
                                            + pow(prob_data[i].z_init - prob_data[j].z_init, 2)));
        }

        // for(int j = 0; j < prob_data[0].x_static_obs_og.rows(); j++){
        //     temp_agent_obs.push_back(sqrt(pow(prob_data[i].x_init - prob_data[i].x_static_obs_og(j, 0), 2) 
        //                                     + pow(prob_data[i].y_init - prob_data[i].y_static_obs_og(j, 0), 2)) 
        //                                     - prob_data[i].a_static_obs_og(j, 0) 
        //                                     + prob_data[i].lx_drone + prob_data[i].buffer);
        // }
    }
    if(temp_inter_agent.size() != 0)
        inter_agent_dist.push_back(*std::min_element(temp_inter_agent.begin(), temp_inter_agent.end()));
    // if(temp_agent_obs.size() != 0)
    //     agent_obs_dist.push_back(*std::min_element(temp_agent_obs.begin(), temp_agent_obs.end()));
    
}

void AMSwarmX :: runIteration(){
    std :: thread agent_thread[num_drone];
    
    auto start = std :: chrono :: high_resolution_clock::now();            
    ros::Rate rate(display_freq);
    ros::Duration(1.5).sleep();

    sim_iter = 0;
    while(ros::ok()){
        
        if(sim_iter < max_iter*max_time/dt && !success){// && !collision_agent && !collision_obstacle){
            for(int i = 0; i < num_drone; i++)
                prob_data[i].max_iter = std :: min(prob_data[i].max_iter+1, max_iter);
            out_space = false;
            
            AMSwarmX :: shareInformation();
            if(sim_iter == 1 || sim_iter == 20 || sim_iter == 50)
            AMSwarmX :: publishDataIteration();
            AMSwarmX :: runAlgorithm();
            if(sim_iter % max_iter == 0){
                AMSwarmX :: applyControl();
                AMSwarmX :: checkCollision();
                AMSwarmX :: checkViolation();
                AMSwarmX :: checkAtGoal();
                AMSwarmX :: calculateDistances();
            }
            mission_time = (sim_iter+1)*dt; 
          
            if(VERBOSE == 2){
                ROS_INFO_STREAM("Simulation time = " << sim_iter);
                ROS_INFO_STREAM("Distance to goals = " << dist_to_goal.transpose());
            }
            sim_iter++;
        }
        else{
            if(!saved){
                auto end = std :: chrono :: high_resolution_clock::now();
                total_time = end - start;
                save_data.close();
                save_data_2.close();
                saved = true;
                AMSwarmX :: saveMetrics();
            }
        }
        rate.sleep();
    }
}

void AMSwarmX :: publishDataIteration(){
    
    
        
    int temp_id = 0;

    agent_trajs.markers.clear();
    agent_startgoals.markers.clear();
    agent_anchors.markers.clear();
    agent_griddatas.markers.clear();
    agent_gridpaths.markers.clear();
    agent_nonconvexsfcs.markers.clear(); 
    agent_nonconvexsfcs.markers.resize(1);

    for(int i = 0; i < num_drone; i++){

        if(i == 0 && !prob_data[i].sfcc){
            agent_nonconvexsfcs.markers[i].header.frame_id = "/map";
            agent_nonconvexsfcs.markers[i].ns = "SFC"+std::to_string(i);
            agent_nonconvexsfcs.markers[i].id = temp_id++;
            agent_nonconvexsfcs.markers[i].type = marker.LINE_STRIP;
            agent_nonconvexsfcs.markers[i].action = marker.ADD;
            agent_nonconvexsfcs.markers[i].scale.x = 0.05;//(2*a_drone+buffer)/2;
            agent_nonconvexsfcs.markers[i].color.a = 1.0;
            // agent_nonconvexsfcs.markers[i].lifetime = ros::Duration(dt);
            agent_nonconvexsfcs.markers[i].color.r = colors[i][0];
            agent_nonconvexsfcs.markers[i].color.g = colors[i][1];
            agent_nonconvexsfcs.markers[i].color.b = colors[i][2];
            agent_nonconvexsfcs.markers[i].pose.orientation.w = 1.0;
           
            
            // for(int j = 0; j < prob_data[i].alpha_sfc.rows(); j++){
            //     double k = prob_data[i].alpha_sfc(j);
            //     octomap::point3d origin(prob_data[i].x_anchor(0), prob_data[i].y_anchor(0), prob_data[i].z_anchor(0));
            //     octomap::point3d direction(cos(k), sin(k), 0.0);
            //     octomap::point3d ray_end;    
            //     double d_sfc;
            //     castRobot(prob_data[i], origin, direction, ray_end, d_sfc);

            //     ray_end = origin + direction * (d_sfc + prob_data[i].distance_to_obs_margin);
            //     agent_nonconvexsfcs.markers[i].points.push_back(buildPoint(ray_end.x(), ray_end.y(), ray_end.z()));
            // }

            // for(double j = prob_data[i].alpha_sfc(prob_data[i].alpha_sfc.rows()-1); j < 2* M_PI + prob_data[i].alpha_sfc(0); j+=2*M_PI/180.0)
            // {
            //     ROS_INFO_STREAM(j);
            //     double k = j;
            //     octomap::point3d origin(prob_data[i].x_anchor(0), prob_data[i].y_anchor(0), prob_data[i].z_anchor(0));
            //     octomap::point3d direction(cos(k), sin(k), 0.0);
            //     octomap::point3d ray_end;    
            //     double d_sfc;
            //     castRobot(prob_data[i], origin, direction, ray_end, d_sfc);

            //     ray_end = origin + direction * (d_sfc + prob_data[i].distance_to_obs_margin);
            //     agent_nonconvexsfcs.markers[i].points.push_back(buildPoint(ray_end.x(), ray_end.y(), ray_end.z()));
            // }
            // double k = prob_data[i].alpha_sfc(0);
            //     octomap::point3d origin(prob_data[i].x_anchor(0), prob_data[i].y_anchor(0), prob_data[i].z_anchor(0));
            //     octomap::point3d direction(cos(k), sin(k), 0.0);
            //     octomap::point3d ray_end;    
            //     double d_sfc;
            //     castRobot(prob_data[i], origin, direction, ray_end, d_sfc);

            //     ray_end = origin + direction * (d_sfc + prob_data[i].distance_to_obs_margin);
            //     agent_nonconvexsfcs.markers[i].points.push_back(buildPoint(ray_end.x(), ray_end.y(), ray_end.z()));
            for(double k = 0; k <= 360.0; k=k+2){
                octomap::point3d origin(prob_data[i].x_anchor(0), prob_data[i].y_anchor(0), prob_data[i].z_anchor(0));
                octomap::point3d direction(cos(k*M_PI/180.0), sin(k*M_PI/180.0), 0.0);
                octomap::point3d ray_end;    
                double d_sfc;
                castRobot(prob_data[i], origin, direction, ray_end, d_sfc);

                ray_end = origin + direction * (d_sfc + prob_data[i].distance_to_obs_margin);
                agent_nonconvexsfcs.markers[i].points.push_back(buildPoint(ray_end.x(), ray_end.y(), ray_end.z()));
            }
        }
        else if(i == 0 && prob_data[i].sfcc){
            if(world == 2){
                auto ells = prob_data[0].decomp_util2.get_ellipsoids();
                // for(int i = 1; i < 0*num_drone; i++){
                //     auto agent_ells = prob_data[i].decomp_util2.get_ellipsoids();
                //     for(auto temp_ells:agent_ells)
                //         ells.push_back(temp_ells);
                // }
                decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(ells);
                es_msg.header.frame_id = "/map";
                es_pub.publish(es_msg);
                
                auto polys = prob_data[0].decomp_util2.get_polyhedrons();
                // for(int i = 1; i < 0*num_drone; i++){
                //     auto agent_polys = prob_data[i].decomp_util2.get_polyhedrons();
                //     for(auto temp_polys:agent_polys)
                //         polys.push_back(temp_polys);
                // }

                decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
                poly_msg.header.frame_id = "/map";
                convexpoly_pub.publish(poly_msg);
            }
            else{
                auto ells = prob_data[0].decomp_util.get_ellipsoids();
                // for(int i = 1; i < 0*num_drone; i++){
                //     auto agent_ells = prob_data[i].decomp_util.get_ellipsoids();
                //     for(auto temp_ells:agent_ells)
                //         ells.push_back(temp_ells);
                // }
                decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(ells);
                es_msg.header.frame_id = "/map";
                es_pub.publish(es_msg);
                
                auto polys = prob_data[0].decomp_util.get_polyhedrons();
                // for(int i = 1; i < 0*num_drone; i++){
                //     auto agent_polys = prob_data[i].decomp_util.get_polyhedrons();
                //     for(auto temp_polys:agent_polys)
                //         polys.push_back(temp_polys);
                // }

                decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
                poly_msg.header.frame_id = "/map";
                convexpoly_pub.publish(poly_msg);
            }
        }
        // crazyflie dae
        marker = visualization_msgs :: Marker();
        marker.header.frame_id = "/map";
        marker.id = temp_id++;
        marker.action = marker.ADD;
        marker.type = marker.MESH_RESOURCE;
        marker.mesh_use_embedded_materials = true;
        if(sim_iter == 0)
        marker.lifetime = ros::Duration(dt);
        marker.mesh_resource = "package://amswarmx/crazyflie/meshes/crazyflie.dae";
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;  
        marker.pose.position.x = agents_x(i, 0) - 0.015;
        marker.pose.position.y = agents_y(i, 0); 
        marker.pose.position.z = agents_z(i, 0); 
        agent_trajs.markers.push_back(marker);  
        

        // ellipse and trajectory
        // for(int j = 0; j < num; j++){
        //     marker = visualization_msgs :: Marker();
        //     marker.header.frame_id = "/map";
        //     marker.id = temp_id++;
        //     marker.type = marker.SPHERE;
        //     marker.action = marker.ADD;
        //     // marker.ns = "Curr"+std::to_string(i);
        //     marker.scale.x = 2*(a_drone) + buffer;
        //     marker.scale.y = 2*(b_drone) + buffer;
        //     marker.scale.z = 2*(c_drone) + buffer;
        //     if(sim_iter == 0 || sim_iter == 50 || sim_iter == 25)
        //         marker.lifetime = ros::Duration(10000*dt);
        //     marker.color.a = 0.5;
        
        //     marker.color.r = colors[i][0];
        //     marker.color.g = colors[i][1];
        //     marker.color.b = colors[i][2];
        //     marker.pose.orientation.w = 1.0;
        //     marker.pose.position.x = agents_x(i, j);
        //     marker.pose.position.y = agents_y(i, j); 
        //     marker.pose.position.z = agents_z(i, j); 
        //     agent_trajs.markers.push_back(marker);  
        // }
        
        marker = visualization_msgs :: Marker();
        for(int j = 0; j < num; j++){
            marker.header.frame_id = "/map";
            marker.ns = "Trajectory";
            marker.id = temp_id++;
            marker.type = marker.LINE_STRIP;
            marker.action = marker.ADD;
            marker.scale.x = (a_drone) + 0*buffer/2.0;
            marker.color.a = 0.5; 
            marker.lifetime = ros::Duration(1000*dt);
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            
            marker.pose.orientation.w = 1.0;
            marker.points.push_back(buildPoint(agents_x(i, j), agents_y(i, j), agents_z(i, j)));
        }
        agent_trajs.markers.push_back(marker);
        
        // start
        marker = visualization_msgs :: Marker();
        marker.header.frame_id = "/map";
        marker.ns = "Start"+std::to_string(i);
        marker.id = temp_id++;
        marker.type = marker.SPHERE;
        marker.action = marker.ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        //marker.lifetime = ros::Duration(dt);
        marker.color.a = 1.0;
        marker.color.r = 0.0;//colors[i][0];
        marker.color.g = 0.0;//colors[i][1];
        marker.color.b = 1.0;//colors[i][2];
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = _init_drone[i][0];
        marker.pose.position.y = _init_drone[i][1]; 
        marker.pose.position.z = _init_drone[i][2];
        marker.text = "S"+std::to_string(i);
        agent_startgoals.markers.push_back(marker);

        // goal                
        marker = visualization_msgs :: Marker();
        marker.header.frame_id = "/map";
        marker.ns = "Goal"+std::to_string(i);
        marker.id = temp_id++;
        marker.type = marker.CUBE;
        marker.action = marker.ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        //marker.lifetime = ros::Duration(dt);
        marker.color.a = 1.0;
        marker.color.r = 0.5;//colors[i][0];
        marker.color.g = 0.0;//colors[i][1];
        marker.color.b = 0.5;//colors[i][2];
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = _goal_drone[i][0];
        marker.pose.position.y = _goal_drone[i][1]; 
        marker.pose.position.z = _goal_drone[i][2];
        // marker.text = "G"+std::to_string(i);
        agent_startgoals.markers.push_back(marker);


        // anchors
        for(int j = 0; j < pieces; j++){
            marker = visualization_msgs :: Marker();
            marker.header.frame_id = "/map";
            marker.id = temp_id++;
            marker.type = marker.TEXT_VIEW_FACING;
            marker.action = marker.ADD;
            marker.ns = "Anchor"+std::to_string(i);
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
            marker.color.a = 1.0;
            marker.color.r = 0.0;//colors[i][0];
            marker.color.g = 0.0;//colors[i][1];
            marker.color.b = 0.0;//colors[i][2];
            marker.pose.orientation.w = 1.0;
            //marker.lifetime = ros::Duration(dt);
            marker.pose.position.x = anchor_x(i, j*num/pieces);
            marker.pose.position.y = anchor_y(i, j*num/pieces); 
            marker.pose.position.z = anchor_z(i, j*num/pieces) +0.0022;
            marker.text = "A";//+std::to_string(i)+std::to_string(j);
            agent_anchors.markers.push_back(marker);

            marker = visualization_msgs :: Marker();
            marker.header.frame_id = "/map";
            marker.id = temp_id++;
            marker.type = marker.CYLINDER;
            marker.action = marker.ADD;
            marker.ns = "Anchor"+std::to_string(i);
            marker.scale.x = 2*(a_drone) + buffer;
            marker.scale.y = 2*(a_drone) + buffer;
            marker.scale.z = 2*(c_drone) + buffer;
            marker.color.a = 1.0;
            marker.color.r = 1.0;//colors[i][0];
            marker.color.g = 1.0;//colors[i][1];
            marker.color.b = 0.0;//colors[i][2];
            marker.pose.orientation.w = 1.0;
            //marker.lifetime = ros::Duration(dt);
            marker.pose.position.x = anchor_x(i, j*num/pieces);
            marker.pose.position.y = anchor_y(i, j*num/pieces); 
            marker.pose.position.z = anchor_z(i, j*num/pieces) +0.0011;  
            agent_anchors.markers.push_back(marker);
        }
        if(sim_iter!=0){
            marker = visualization_msgs :: Marker();
            marker.header.frame_id = "/map";
            marker.id = temp_id++;
            marker.type = marker.TEXT_VIEW_FACING;
            marker.action = marker.ADD;
            marker.ns = "InterGoal"+std::to_string(i);
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
             //marker.lifetime = ros::Duration(dt);
            marker.color.a = 1.0;
            marker.color.r = 0.0;//colors[i][0];
            marker.color.g = 0.0;//colors[i][1];
            marker.color.b = 0.0;//colors[i][2];
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = prob_data[i].x_goal;
            marker.pose.position.y = prob_data[i].y_goal;  
            marker.pose.position.z = prob_data[i].z_goal+0.0022;
            marker.text = "IG";//+std::to_string(i);
            agent_anchors.markers.push_back(marker);

            marker = visualization_msgs :: Marker();
            marker.header.frame_id = "/map";
            marker.id = temp_id++;
            marker.type = marker.CYLINDER;
            marker.action = marker.ADD;
            marker.ns = "InterGoal"+std::to_string(i);
            marker.scale.x = 2*(a_drone) + buffer;
            marker.scale.y = 2*(a_drone) + buffer;
            marker.scale.z = 2*(c_drone) + buffer;
             //marker.lifetime = ros::Duration(dt);
            marker.color.a = 1.0;
            marker.color.r = 0.5;//colors[i][0];
            marker.color.g = 0.0;//colors[i][1];
            marker.color.b = 0.5;//colors[i][2];
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = prob_data[i].x_goal;
            marker.pose.position.y = prob_data[i].y_goal; 
            marker.pose.position.z = prob_data[i].z_goal +0.0011;
            agent_anchors.markers.push_back(marker);
        }
        
        // path
        agent_paths.markers[i].header.frame_id = "/map";
        agent_paths.markers[i].ns = "Path"+std::to_string(i);
        agent_paths.markers[i].id = temp_id++;
        agent_paths.markers[i].type = marker.LINE_STRIP;
        agent_paths.markers[i].action = marker.ADD;
        //marker.lifetime = ros::Duration(dt);
        agent_paths.markers[i].scale.x = 0.02;
        agent_paths.markers[i].color.a = 1.0;
        agent_paths.markers[i].color.r = colors[i][0];
        agent_paths.markers[i].color.g = colors[i][1];
        agent_paths.markers[i].color.b = colors[i][2];
        agent_paths.markers[i].pose.orientation.w = 1.0;
        agent_paths.markers[i].points.push_back(buildPoint(agents_x(i, 0), agents_y(i, 0), agents_z(i, 0)));
        if(sim_iter == 0)
            agent_paths.markers[i].points.push_back(buildPoint(agents_x(i, 0), agents_y(i, 0), agents_z(i, 0)));


        // gridpath
        for(int k = 0; k < prob_data[i].grid_path.size(); k++){
            marker = visualization_msgs :: Marker();
            marker.header.frame_id = "/map";
            marker.ns = "GridPath"+std::to_string(i);
            marker.id = temp_id++;
            marker.type = marker.SPHERE;
            marker.action = marker.ADD;
            marker.scale.x = 0.06;
            marker.scale.y = 0.06;
            marker.scale.z = 0.06;
            //marker.lifetime = ros::Duration(dt);
            marker.color.a = 0.5;
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = prob_data[i].grid_path[k][0];
            marker.pose.position.y = prob_data[i].grid_path[k][1]; 
            marker.pose.position.z = prob_data[i].grid_path[k][2];
            agent_gridpaths.markers.push_back(marker);
        }

        // vels and accs
        if(i == 0){
            if(sim_iter == 0){
                agent_vels_x.data[i] = 0.0;
                agent_accs_x.data[i] = 0.0;
                agent_vels_y.data[i] = 0.0;
                agent_accs_y.data[i] = 0.0;
                agent_vels_z.data[i] = 0.0;
                agent_accs_z.data[i] = 0.0;
            }
            else{
                agent_vels_x.data[i] = prob_data[i].vx_init;
                agent_accs_x.data[i] = prob_data[i].ax_init;
                agent_vels_y.data[i] = prob_data[i].vy_init;
                agent_accs_y.data[i] = prob_data[i].ay_init;
                agent_vels_z.data[i] = prob_data[i].vz_init;
                agent_accs_z.data[i] = prob_data[i].az_init;
            }
        }
    }
    traj_pub.publish(agent_trajs);
    startgoal_pub.publish(agent_startgoals);
    anchor_pub.publish(agent_anchors);
    path_pub.publish(agent_paths);
    if(sim_iter!=0){
        gridpath_pub.publish(agent_gridpaths);
        griddata_pub.publish(agent_griddatas);
        nonconvexsfc_pub.publish(agent_nonconvexsfcs);
    }
    vel_x_pub.publish(agent_vels_x);
    vel_y_pub.publish(agent_vels_y);
    vel_z_pub.publish(agent_vels_z);
    acc_x_pub.publish(agent_accs_x);
    acc_y_pub.publish(agent_accs_y);
    acc_z_pub.publish(agent_accs_z);

    
    
    for(int i = 0; i < agent_boundaries.markers.size(); i++){
        agent_boundaries.markers[i].points.clear();
        agent_boundaries.markers[i].header.frame_id = "/map";
        agent_boundaries.markers[i].id = temp_id++;
        agent_boundaries.markers[i].type = marker.LINE_STRIP;
        agent_boundaries.markers[i].action = marker.ADD;
        agent_boundaries.markers[i].scale.x = 0.05;
        agent_boundaries.markers[i].color.a = 1.0;
        agent_boundaries.markers[i].color.r = 0.0;
        agent_boundaries.markers[i].color.g = 0.0;
        agent_boundaries.markers[i].color.b = 0.0;
        agent_boundaries.markers[i].pose.orientation.w = 1.0;
    }

    agent_boundaries.markers[0].points.push_back(buildPoint(x_min, y_min, z_min));
    agent_boundaries.markers[0].points.push_back(buildPoint(x_min, y_max, z_min));
    agent_boundaries.markers[0].points.push_back(buildPoint(x_max, y_max, z_min));
    agent_boundaries.markers[0].points.push_back(buildPoint(x_max, y_min, z_min));
    agent_boundaries.markers[0].points.push_back(buildPoint(x_min, y_min, z_min));

    agent_boundaries.markers[1].points.push_back(buildPoint(x_min, y_min, z_max));
    agent_boundaries.markers[1].points.push_back(buildPoint(x_min, y_max, z_max));
    agent_boundaries.markers[1].points.push_back(buildPoint(x_max, y_max, z_max));
    agent_boundaries.markers[1].points.push_back(buildPoint(x_max, y_min, z_max));
    agent_boundaries.markers[1].points.push_back(buildPoint(x_min, y_min, z_max));
    
    agent_boundaries.markers[2].points.push_back(buildPoint(x_max, y_max, z_min));
    agent_boundaries.markers[2].points.push_back(buildPoint(x_max, y_max, z_max));

    agent_boundaries.markers[3].points.push_back(buildPoint(x_max, y_min, z_min));
    agent_boundaries.markers[3].points.push_back(buildPoint(x_max, y_min, z_max));

    agent_boundaries.markers[4].points.push_back(buildPoint(x_min, y_max, z_min));
    agent_boundaries.markers[4].points.push_back(buildPoint(x_min, y_max, z_max));

    agent_boundaries.markers[5].points.push_back(buildPoint(x_min, y_min, z_min));
    agent_boundaries.markers[5].points.push_back(buildPoint(x_min, y_min, z_max));
 
    boundary_pub.publish(agent_boundaries);

}



void AMSwarmX :: saveMetrics(){
    
    if(success && !collision_agent && !collision_obstacle){
        
        save_data.open(package_path+"/data/" + result_name + "_sim_info.txt");
        // save_data_2.open(package_path+"/data/sim_anchor.txt");

        save_data << num_drone << " " << num_obs << " " << dt << "\n"; 
        save_data << a_drone << " " << b_drone << " " << c_drone << "\n"; 
        save_data << t_plan << " " << pieces << " " << degree << "\n";
        save_data << params["x_lim"][0] << " " << params["y_lim"][0] << " " << params["z_lim"][0] << "\n";
        save_data << params["x_lim"][1] << " " << params["y_lim"][1] << " " << params["z_lim"][1] << "\n";

        for(int i = 0; i < num_drone; i++) save_data << _init_drone[i][0] << " " << _init_drone[i][1] << " " << _init_drone[i][2] << "\n";
        for(int i = 0; i < num_drone; i++) save_data << _goal_drone[i][0] << " " << _goal_drone[i][1] << " " << _goal_drone[i][2] << "\n";
        
        for(int i = 0; i < num_obs; i++) save_data << _pos_static_obs[i][0] << " " << _pos_static_obs[i][1] << " " << _pos_static_obs[i][2] << "\n"; 
        for(int i = 0; i < num_obs; i++) save_data << _dim_static_obs[i][0] << " " << _dim_static_obs[i][1] << " " << _dim_static_obs[i][2] << "\n";

        save_data.close();
        
        save_data.open(package_path+"/data/" + result_name + "_sim_results.txt");
        for(int i = 0; i < num_drone; i++){
            double smoothness = 0, arc_length = 0;
            for(int j = 0; j < prob_data[i].smoothness.size(); j++){
                smoothness += pow(prob_data[i].smoothness[j], 2);
                arc_length += prob_data[i].arc_length[j];
            }
            smoothness = sqrt(smoothness);

            smoothness_agent.push_back(smoothness);
            traj_length_agent.push_back(arc_length);
        }

        save_data << sim_iter << "\n";          // 1. SIM ITERATIONS

        avg_smoothness=0.0; 
        avg_traj_length=0.0; 
        min_inter_agent_dist=10000.0;
        avg_inter_agent_dist=0.0;
        min_agent_obs_dist=10000.0;
        avg_agent_obs_dist=0.0;
        avg_comp_time_per_agent = 0.0;
        avg_comp_time = 0.0 ;

        for(int i = 0; i < comp_time_agent.size(); i++){
            avg_comp_time_per_agent += comp_time_agent[i]; 
            save_data << comp_time_agent[i] << "\n";        // 2. COMPUTE TIME PER AGENT
        }
        for(int i = 0; i < smoothness_agent.size(); i++){ 
            save_data << smoothness_agent[i] << "\n";       // 3. SMOOTHNESS
            avg_smoothness += smoothness_agent[i];
        };
        for(int i = 0; i < traj_length_agent.size(); i++){
            save_data << traj_length_agent[i] << "\n";      // 4. TRAJECTORY LENGTH
            avg_traj_length += traj_length_agent[i];
        }
        for(int i = 0; i < inter_agent_dist.size(); i++){
            save_data << inter_agent_dist[i] << "\n";      // 5. INTER-AGENT DIST
            avg_inter_agent_dist += inter_agent_dist[i];

            if(min_inter_agent_dist > inter_agent_dist[i])
                min_inter_agent_dist = inter_agent_dist[i];
        }
        for(int i = 0; i < agent_obs_dist.size(); i++){
            save_data << agent_obs_dist[i] << "\n";      // 6. AGENT-OBS DIST
            avg_agent_obs_dist += agent_obs_dist[i];

            if(min_agent_obs_dist > agent_obs_dist[i])
                min_agent_obs_dist = agent_obs_dist[i];
        }
        
        if(smoothness.size()!=0) avg_smoothness = avg_smoothness / (smoothness.size());
        if(traj_length_agent.size()!=0) avg_traj_length = avg_traj_length / (traj_length_agent.size());
        if(inter_agent_dist.size()!=0) avg_inter_agent_dist = avg_inter_agent_dist / (inter_agent_dist.size());
        if(agent_obs_dist.size()!=0) avg_agent_obs_dist = avg_agent_obs_dist / (agent_obs_dist.size());
        if(comp_time_agent.size()!=0){ avg_comp_time_per_agent = avg_comp_time_per_agent / (comp_time_agent.size());     
            avg_comp_time = avg_comp_time_per_agent * num_drone;
        }

        save_data << mission_time << "\n";                          // 7. MISSION TIME
        save_data << avg_comp_time << "\n";
        save_data << avg_comp_time_per_agent << "\n";
        save_data << avg_smoothness << "\n";
        save_data << avg_traj_length << "\n";
        save_data << avg_inter_agent_dist << "\n";
        save_data << min_inter_agent_dist << "\n";
        save_data << avg_agent_obs_dist << "\n";
        save_data << min_agent_obs_dist << "\n";
        save_data.close();    

        ROS_INFO_STREAM("________ SUCCESS! _________");
        ROS_INFO_STREAM("Mission completion time = " << mission_time << " s");
        ROS_INFO_STREAM("Average time and freq to compute = " << avg_comp_time << " s, " << 1/avg_comp_time << " Hz");
        ROS_INFO_STREAM("Average run time per agent = " << avg_comp_time_per_agent << " s");
        ROS_INFO_STREAM("Average smoothness = " << avg_smoothness << " ms^-2");
        ROS_INFO_STREAM("Average trajectory length = " << avg_traj_length << " m");
        ROS_INFO_STREAM("Average inter-agent dist = " << avg_inter_agent_dist << " m");
        ROS_INFO_STREAM("Smallest inter-agent dist = " << min_inter_agent_dist << " m");
        ROS_INFO_STREAM("Average obs-agent dist = " << avg_agent_obs_dist << " m");
        ROS_INFO_STREAM("Smallest obs-agent dist = " << min_agent_obs_dist << " m");
    }
    else{
        success = false;
        ROS_INFO_STREAM("________ FAILURE! _________");
        if(collision_agent)
            ROS_WARN_STREAM("Inter-agent collision");
        else if(collision_obstacle)
            ROS_WARN_STREAM("Obstacle-agent collision");
        else
            ROS_WARN_STREAM("Goal not Reached");
    }    

    if(VERBOSE){
        ROS_INFO_STREAM("________ CONVERGENCE DETAILS! _________");
        for(int i = 0; i < num_drone; i++){
            ROS_INFO_STREAM("Percentage convergence failure " << " Drone " << i << " =" << ((double)prob_data[i].not_converged_count/(double)sim_iter)*100.0);
        }
    }
    delete[] prob_data;
}


geometry_msgs::Point buildPoint(double x, double y, double z){
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;

        return p;
}
std::vector<double> generateRandomPosition(double x_min, double x_max, 
                                        double y_min, double y_max, 
                                        double z_min, double z_max) {
    static std::mt19937 gen(0);
    static std::uniform_real_distribution<double> dis_x(x_min, x_max);
    static std::uniform_real_distribution<double> dis_y(y_min, y_max);
    static std::uniform_real_distribution<double> dis_z(z_min, z_max);
    return {dis_x(gen), dis_y(gen), dis_z(gen)};
}

bool isDistanceGreaterThanR(const std::vector<double>& position, 
                            const std::vector<std::vector<double>>& positions, 
                            double r) {
    for (const auto& old_position : positions) {
        double distance = 0;
        for (int i = 0; i < 3; i++) {
            distance += std::pow(position[i] - old_position[i], 2);
        }
        if (std::sqrt(distance) < r) {
            return false;
        }
    }
    return true;
}

std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> generateInitialAndGoalPositions(int num_drone, 
                                double x_min, double x_max, 
                                double y_min, double y_max, 
                                double z_min, double z_max, 
                                double r, std::shared_ptr<DynamicEDTOctomap> distmap_obj) {
    std::vector<std::vector<double>> init_drone;
    std::vector<std::vector<double>> goal_drone;
    while (init_drone.size() < num_drone) {
        auto position = generateRandomPosition(x_min, x_max, y_min, y_max, z_min, z_max);
        if (isDistanceGreaterThanR(position, init_drone, 2*r) &&  distmap_obj->getDistance(octomap::point3d(position[0], position[1], position[2])) >= 1.5*r) {
            init_drone.push_back(position);
        }
    }
    while (goal_drone.size() < num_drone) {
        auto position = generateRandomPosition(x_min, x_max, y_min, y_max, z_min, z_max);
        if (isDistanceGreaterThanR(position, goal_drone, 2*r) && 
            isDistanceGreaterThanR(position, init_drone, 2*r) && distmap_obj->getDistance(octomap::point3d(position[0], position[1], position[2])) >= 1.5*r) {
            goal_drone.push_back(position);
        }
    }
    return {init_drone, goal_drone};
}

// Eigen::Vector3f computeQuaternions(Eigen :: Vector3f thrust){
//    double yaw = 0;
//     // Desired attitude
//     Eigen::Vector3f Z_b = thrust.normalized();

//     Eigen::Vector3f X_c(cos(yaw), sin(yaw), 0);
//     Eigen::Vector3f Y_b = Z_b.cross(X_c).normalized();
//     Eigen::Vector3f X_b = Y_b.cross(Z_b);

//     // Rotation matrix from body frame to world frame
//     Eigen::Matrix3f R_wb; // Note: We changed the matrix name to R_wb
//     R_wb << X_b(0), Y_b(0), Z_b(0),
//             X_b(1), Y_b(1), Z_b(1),
//             X_b(2), Y_b(2), Z_b(2);

//     // Euler angle, Z-X-Y convention
//     Eigen::Vector3f euler;

//     euler[0] = asin(-R_wb(1, 2)); // Negate the sign here for body frame to world frame
//     euler[1] = atan2(R_wb(0, 2), R_wb(2, 2));
//     euler[2] = atan2(R_wb(1, 0), R_wb(1, 1));

//     return euler;
// }