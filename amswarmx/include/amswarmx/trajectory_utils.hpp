#pragma once

#include <ros/package.h>
#include <ros/ros.h>

#include <vector>
#include <chrono>
#include <iostream>

#include <eigen3/Eigen/Dense>
#include "yaml-cpp/yaml.h"
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_geometry/geometric_utils.h>


struct five_var
{
    Eigen :: ArrayXXd a, b, c, d, e;
};    
struct probData
{
    bool jerk_snap_constraints, axis_wise, free_space, sfcc, jps, castray, path_replan, converged_flag;
    int not_converged_count;

    int num, num_up, num_static_obs, num_drone, num_obs, nvar, id_badge, unify_obs,
        max_iter, mpc_step, kappa, world, pieces, degree, waypoint_index, prev_waypoint_index, anchor_index;

    double grid_margin, grid_resolution, visibility_margin, distance_to_obs_margin;
    double t_plan, vel_max, acc_max, jerk_max, snap_max, max_sim_time, dt, dist_to_goal, total_dist;
    double weight_smoothness, weight_goal, weight_smoothness_og, weight_goal_og;
    double rho_static_obs, rho_drone, rho_vel, rho_acc, rho_jerk, rho_snap, rho_ineq, rho_sfc;
    double rho_static_obs_max, rho_drone_max, rho_vel_max, rho_acc_max, rho_jerk_max, rho_snap_max, rho_ineq_max, rho_sfc_max;
    double delta_static_obs, delta_drone, delta_vel, delta_acc, delta_jerk, delta_snap, delta_ineq, delta_aggressive, delta_sfc;
    double buffer, prox_obs, prox_agent, dist_stop;
    double lx_drone, ly_drone, lz_drone;

    double x_min, y_min, z_min,
            x_max, y_max, z_max,
            x_init, y_init, z_init,
            x_goal, y_goal, z_goal, 
            prev_x_goal, prev_y_goal, prev_z_goal, 
            xf_goal, yf_goal, zf_goal,
            vx_init, vy_init, vz_init,
            ax_init, ay_init, az_init;
    
    double gravity, f_min, f_max;
    bool use_thrust_values;
    double thresold, gamma;

    double world_resolution;
    
    double res_x_static_obs_norm, res_y_static_obs_norm,
            res_x_drone_norm, res_y_drone_norm, res_z_drone_norm,
            res_x_vel_norm, res_y_vel_norm, res_z_vel_norm,
            res_x_acc_norm, res_y_acc_norm, res_z_acc_norm,
            res_x_jerk_norm, res_y_jerk_norm, res_z_jerk_norm,
            res_x_snap_norm, res_y_snap_norm, res_z_snap_norm,
            res_x_ineq_norm, res_y_ineq_norm, res_z_ineq_norm,
            res_x_sfc_norm, res_y_sfc_norm, res_z_sfc_norm;

    Eigen :: ArrayXd grid_x, grid_y, grid_z;

    Eigen :: ArrayXXd agents_x, agents_y, agents_z;
    Eigen :: ArrayXXd cost_smoothness, cost_goal, cost_vel, cost_acc, cost_jerk, cost_snap, cost_ineq, cost_sfc, I;

    Eigen :: ArrayXXd P, Pdot, Pddot, Pdddot, Pddddot;
    Eigen :: ArrayXXd W, Wdot, Wddot, Wdddot, Wddddot;  
    Eigen :: ArrayXXd P_up, Pdot_up, Pddot_up, Pdddot_up, Pddddot_up;
    Eigen :: ArrayXXd W_up, Wdot_up, Wddot_up, Wdddot_up, Wddddot_up;

    Eigen :: ArrayXXd A_eq, A_ineq, A_v_ineq, A_a_ineq, A_j_ineq, A_s_ineq, A_init, A_conti, A_static_obs, A_drone, A_x_cp, A_y_cp, A_z_cp;

    Eigen :: ArrayXXd b_cp, B_x_cp, B_y_cp, B_z_cp, s_cp;

    Eigen :: ArrayXXd B_x_ineq, B_y_ineq, B_z_ineq;
    Eigen :: ArrayXXd b_x_ineq, b_y_ineq, b_z_ineq;
    Eigen :: ArrayXXd s_x_ineq, s_y_ineq, s_z_ineq;

    Eigen :: ArrayXXd B_vx_ineq, B_vy_ineq, B_vz_ineq;
    Eigen :: ArrayXXd b_vx_ineq, b_vy_ineq, b_vz_ineq;
    Eigen :: ArrayXXd b_x_vel, b_y_vel, b_z_vel;
    Eigen :: ArrayXXd s_vx_ineq, s_vy_ineq, s_vz_ineq;

    Eigen :: ArrayXXd B_ax_ineq, B_ay_ineq, B_az_ineq;
    Eigen :: ArrayXXd b_ax_ineq, b_ay_ineq, b_az_ineq;
    Eigen :: ArrayXXd b_x_acc, b_y_acc, b_z_acc;
    Eigen :: ArrayXXd s_ax_ineq, s_ay_ineq, s_az_ineq;

    Eigen :: ArrayXXd B_jx_ineq, B_jy_ineq, B_jz_ineq;
    Eigen :: ArrayXXd b_jx_ineq, b_jy_ineq, b_jz_ineq;
    Eigen :: ArrayXXd b_x_jerk, b_y_jerk, b_z_jerk;
    Eigen :: ArrayXXd s_jx_ineq, s_jy_ineq, s_jz_ineq;

    Eigen :: ArrayXXd B_sx_ineq, B_sy_ineq, B_sz_ineq;
    Eigen :: ArrayXXd b_sx_ineq, b_sy_ineq, b_sz_ineq;
    Eigen :: ArrayXXd b_x_snap, b_y_snap, b_z_snap;
    Eigen :: ArrayXXd s_sx_ineq, s_sy_ineq, s_sz_ineq;

    Eigen :: ArrayXXd b_x_eq, b_y_eq, b_z_eq;
    Eigen :: ArrayXXd b_x_init, b_y_init, b_z_init;
    Eigen :: ArrayXXd b_x_conti, b_y_conti, b_z_conti;
    Eigen :: ArrayXXd b_x_static_obs, b_y_static_obs;
    Eigen :: ArrayXXd b_x_drone, b_y_drone, b_z_drone;
    

    Eigen :: ArrayXXd d_static_obs, d_drone, d_vel, d_acc, d_jerk, d_snap, d_drone_old, d_static_obs_old, d_sfc; 
    Eigen :: ArrayXXd alpha_static_obs, alpha_drone, alpha_vel, alpha_acc, alpha_jerk, alpha_snap, alpha_sfc, beta_sfc;
    Eigen :: ArrayXXd beta_drone, beta_vel, beta_acc, beta_jerk, beta_snap; 

    Eigen :: ArrayXXd x_static_obs, y_static_obs, z_static_obs;
    Eigen :: ArrayXXd x_static_obs_og, y_static_obs_og, z_static_obs_og;
    Eigen :: ArrayXXd x_drone, y_drone, z_drone;
    Eigen :: ArrayXXd a_static_obs, b_static_obs, c_static_obs;
    Eigen :: ArrayXXd a_static_obs_og, b_static_obs_og, c_static_obs_og;  
    Eigen :: ArrayXXd a_drone, b_drone, c_drone;

    Eigen :: ArrayXXd x, y, z,
                        prev_x, prev_y, prev_z,
                        xdot, ydot, zdot,
                        xddot, yddot, zddot,
                        xdddot, ydddot, zdddot,
                        xddddot, yddddot, zddddot;
    Eigen :: ArrayXXd x_anchor, y_anchor, z_anchor;

    Eigen :: ArrayXXd x_up, y_up, z_up,
                        xdot_up, ydot_up, zdot_up,
                        xddot_up, yddot_up, zddot_up;
    
    Eigen :: ArrayXXd x_ref, y_ref, z_ref;
    Eigen :: ArrayXXd lamda_x, lamda_y, lamda_z;
    Eigen :: ArrayXXd lamda_x_ineq, lamda_y_ineq, lamda_z_ineq;        

    std :: vector<double> smoothness, arc_length, 
                            inter_agent_dist, agent_obs_dist, 
                            inter_agent_dist_min, agent_obs_dist_min;
    std :: vector<std :: vector<double>> world_dimension;
    std :: vector<signed char> grid_data;
    std :: vector<std :: vector<double>> pos_static_obs, dim_static_obs;
    std :: vector<std :: vector<double>> anchor_points, grid_path;

    YAML :: Node params;

    octomap::OcTree* octree_ptr;
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    std::shared_ptr<JPS::OccMapUtil> map_util;
    std::shared_ptr<JPS::VoxelMapUtil> map_util_;

    vec_Vec3f decomp_obs; 
    vec_Vec2f decomp_obs2;

    EllipsoidDecomp3D decomp_util;
    EllipsoidDecomp2D decomp_util2;
    
    std :: vector<std :: vector <double>> convex_poly; 
};


five_var bernsteinCoeff(double n, double tmin, double tmax, Eigen :: ArrayXXd t_actual, int num);
five_var computeBernstein(Eigen :: ArrayXXd tot_time, double t_plan, int num, int pieces, int degree);
Eigen :: ArrayXXd stack(Eigen :: ArrayXXd arr1, Eigen :: ArrayXXd arr2, char ch);
Eigen :: ArrayXXd reshape(Eigen :: ArrayXXd x, uint32_t r, uint32_t c);
Eigen :: ArrayXXd arctan2(Eigen :: ArrayXXd arr1, Eigen :: ArrayXXd arr2);
Eigen :: ArrayXXd diff(Eigen :: ArrayXXd arr);
Eigen :: ArrayXXd block_diag(Eigen :: ArrayXXd arr1, Eigen :: ArrayXXd arr2);