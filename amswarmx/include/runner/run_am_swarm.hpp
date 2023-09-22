#pragma once
#include <fstream>
#include <chrono>
#include <sstream>
#include <thread>
#include <random>

// #include "jsoncpp/json/json.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <decomp_ros_utils/data_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Empty.h"
#include "amswarmx/run_trajectory_optimizer.hpp"


class AMSwarmX{
    public:
        bool success;
        bool visualize_am;
        bool sfcc;
        
        int num_drone;
        int trials;
        bool write_files;
        std :: string world_file_name, json_name, result_name;
        probData *prob_data;

        AMSwarmX();
        void runSimulation();
        void runIteration();
        void saveMetrics();

    private:    
        ros::NodeHandle nh;
        ros::Publisher traj_pub;
        ros::Publisher startgoal_pub;
        ros::Publisher anchor_pub;
        ros::Publisher path_pub;
        ros::Publisher gridpath_pub;
        ros::Publisher griddata_pub;
        ros::Publisher boundary_pub;
        ros::Publisher vel_x_pub, vel_y_pub, vel_z_pub;
        ros::Publisher acc_x_pub, acc_y_pub, acc_z_pub;
        ros::Publisher convexpoly_pub;
        ros::Publisher es_pub;
        ros::Publisher nonconvexsfc_pub;

        visualization_msgs :: MarkerArray agent_trajs, agent_startgoals, agent_anchors, 
                                        agent_paths, agent_gridpaths, agent_griddatas, 
                                        agent_boundaries, agent_convexpolys, agent_nonconvexsfcs;
        visualization_msgs :: Marker marker;
        std_msgs :: Float64MultiArray agent_vels_x, agent_vels_y, agent_vels_z;
        std_msgs :: Float64MultiArray agent_accs_x, agent_accs_y, agent_accs_z; 
        
        std :: ofstream save_data, save_data_2;

        std :: string package_path;
        YAML :: Node params;

        double world_resolution;
        std::vector<std::vector<double>> world_dimension;

        int world;
        int camera_follow_drone;
        int display_freq;
        int cluster;
        int config_num;
        int VERBOSE;
        int num;
        
        int sim_iter;
        int pieces;
        int degree;

        std::vector<std::vector<double>> colors;
        bool free_space;
        bool display_sim;

        double a_drone;
        double b_drone;
        double c_drone;
        double buffer;

        double max_time;
        int max_iter;
        double t_plan;
        double dist_stop;
        double dt;

        bool random_config;
        bool out_space;
        bool saved;
        

        double x_max, y_max, z_max,
                x_min, y_min, z_min;
                
        bool collision_agent, collision_obstacle;

        double avg_smoothness, avg_traj_length, 
              min_inter_agent_dist, avg_inter_agent_dist,
              min_agent_obs_dist, avg_agent_obs_dist,
              avg_comp_time_per_agent, avg_comp_time;
        
        double mission_time;

        std :: chrono :: duration<double, std::milli> total_time;

        Eigen :: ArrayXXd agents_x, anchor_x,
                        agents_y, anchor_y,
                        agents_z, anchor_z;

        Eigen :: ArrayXd smoothness,
                        arc_length,
                        dist_to_goal;

        std :: stringstream folder_name;
        std :: vector<std :: vector<double>> _init_drone;
        std :: vector<std :: vector<double>> _goal_drone;

        std :: vector<std :: vector<double>> _pos_static_obs;
        std :: vector<std :: vector<double>> _dim_static_obs;

        std :: vector<std :: vector<std :: vector<double>>> _anchor_points;
        std :: vector <double> smoothness_agent, traj_length_agent, comp_time_agent, inter_agent_dist, agent_obs_dist;
        int num_obs, num_obs_2;    

        void shareInformation();
        void runAlgorithm();
        void applyControl();
        void checkCollision();
        void checkAtGoal();
        void checkViolation();
        void calculateDistances();
        void publishDataAlgorithm();
        void publishDataIteration();
        void solveCluster(int start_index);
};


geometry_msgs::Point buildPoint(double x, double y, double z);
Eigen::Vector3f computeQuaternions(Eigen :: Vector3f thrust);
std::vector<double> generateRandomPosition(double x_min, double x_max, 
                                        double y_min, double y_max, 
                                        double z_min, double z_max);
bool isDistanceGreaterThanR(const std::vector<double>& position, 
                            const std::vector<std::vector<double>>& positions, 
                            double r);

std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> generateInitialAndGoalPositions(int num_drone, 
                                    double x_min, double x_max, 
                                    double y_min, double y_max, 
                                    double z_min, double z_max, 
                                    double r,  std::shared_ptr<DynamicEDTOctomap> distmap_obj);