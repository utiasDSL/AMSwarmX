#include "amswarmx/run_trajectory_optimizer.hpp"

void checkResiduals(probData &prob_data, int VERBOSE)
{
	double thresold = prob_data.thresold;
	
	prob_data.converged_flag = true;
	if(prob_data.res_x_static_obs_norm > thresold || prob_data.res_y_static_obs_norm > thresold){
		ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
		ROS_WARN_STREAM("Static Obstacle Constraints = " << prob_data.res_x_static_obs_norm << " " << prob_data.res_y_static_obs_norm);
		prob_data.converged_flag = false;
	}
	if(prob_data.res_x_sfc_norm > thresold || prob_data.res_y_sfc_norm > thresold || prob_data.res_z_sfc_norm > thresold){
		ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
		ROS_WARN_STREAM("SFC Constraints = " << prob_data.res_x_sfc_norm << " " << prob_data.res_y_sfc_norm << " " << prob_data.res_z_sfc_norm);
		prob_data.converged_flag = false;
	}
	if(prob_data.res_x_drone_norm > thresold || prob_data.res_y_drone_norm > thresold || prob_data.res_z_drone_norm > thresold){
		ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
		ROS_WARN_STREAM("Inter-Agent Collision Constraints = " << prob_data.res_x_drone_norm << " " << prob_data.res_y_drone_norm << " " << prob_data.res_z_drone_norm);
		prob_data.converged_flag = false;
	}
	if(prob_data.res_x_vel_norm > thresold || prob_data.res_y_vel_norm > thresold || prob_data.res_z_vel_norm > thresold){
		ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
		ROS_WARN_STREAM("Velocity Constraints = " << prob_data.res_x_vel_norm << " " << prob_data.res_y_vel_norm << " " << prob_data.res_z_vel_norm);
		prob_data.converged_flag = false;
	}
	if(prob_data.res_x_acc_norm > thresold || prob_data.res_y_acc_norm > thresold || prob_data.res_z_acc_norm > thresold){
		ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
		ROS_WARN_STREAM("Acceleration Constraints = " << prob_data.res_x_acc_norm << " " << prob_data.res_y_acc_norm << " " << prob_data.res_z_acc_norm);
		prob_data.converged_flag = false;
	}
	if(prob_data.res_x_jerk_norm > thresold || prob_data.res_y_jerk_norm > thresold || prob_data.res_z_jerk_norm > thresold){
		ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
		ROS_WARN_STREAM("Jerk Constraints = " << prob_data.res_x_jerk_norm << " " << prob_data.res_y_jerk_norm << " " << prob_data.res_z_jerk_norm);
		prob_data.converged_flag = false;
	}
	if(prob_data.res_x_snap_norm > thresold || prob_data.res_y_snap_norm > thresold || prob_data.res_z_snap_norm > thresold){
		ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
		ROS_WARN_STREAM("Snap Constraints = " << prob_data.res_x_snap_norm << " " << prob_data.res_y_snap_norm << " " << prob_data.res_z_snap_norm);
		prob_data.converged_flag = false;
	}
	if(prob_data.res_x_ineq_norm > thresold || prob_data.res_y_ineq_norm > thresold || prob_data.res_z_ineq_norm > thresold){
		ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
		ROS_WARN_STREAM("Workspace Constraints = " << prob_data.res_x_ineq_norm << " " << prob_data.res_y_ineq_norm << " " << prob_data.res_z_ineq_norm);
		prob_data.converged_flag = false;
	}

	if(!prob_data.converged_flag){
		prob_data.not_converged_count++;
	}
}


void initializeOptimizer(probData &prob_data, int VERBOSE){
	// @ Load parameters from config file
	YAML :: Node params = prob_data.params;
	
	std :: vector<std :: vector<double>> _pos_static_obs = prob_data.pos_static_obs;
	std :: vector<std :: vector<double>> _dim_static_obs = prob_data.dim_static_obs;
	
	prob_data.num_static_obs = _pos_static_obs.size();
	
	Eigen :: ArrayXd x_static_obs(prob_data.num_static_obs); 
	Eigen :: ArrayXd y_static_obs(prob_data.num_static_obs);
	Eigen :: ArrayXd z_static_obs(prob_data.num_static_obs);

	Eigen :: ArrayXd a_static_obs(prob_data.num_static_obs); 
	Eigen :: ArrayXd b_static_obs(prob_data.num_static_obs);
	Eigen :: ArrayXd c_static_obs(prob_data.num_static_obs);
	
	for(int i = 0; i < prob_data.num_static_obs; i++){
		x_static_obs(i) = _pos_static_obs[i][0];
		y_static_obs(i) = _pos_static_obs[i][1];
		z_static_obs(i) = _pos_static_obs[i][2];
	
		a_static_obs(i) = _dim_static_obs[i][0];
		b_static_obs(i) = _dim_static_obs[i][1];
		c_static_obs(i) = _dim_static_obs[i][2];
	}

	prob_data.castray = params["castray"].as<bool>();
	prob_data.free_space = params["free_space"].as<bool>();
	
	if(prob_data.free_space){
		prob_data.num_static_obs = 0;
	}
	
	std :: vector<double> _x_lim = params["x_lim"].as<std::vector<double>>();
	std :: vector<double> _y_lim = params["y_lim"].as<std::vector<double>>();
	std :: vector<double> _z_lim = params["z_lim"].as<std::vector<double>>();

	
	prob_data.world = params["world"].as<int>();
	prob_data.num = params["num"].as<int>();
	
	prob_data.t_plan = params["t_plan"].as<int>();
	prob_data.num_up = params["num_up"].as<int>();
	prob_data.dist_stop = params["dist_stop"].as<double>();
	prob_data.kappa = params["kappa"].as<int>();
	prob_data.pieces = params["pieces"].as<int>();
	prob_data.degree = params["degree"].as<int>();
	prob_data.sfcc = params["sfcc"].as<bool>();
	prob_data.jps = params["jps"].as<bool>();
	prob_data.path_replan = params["path_replan"].as<bool>();
	prob_data.grid_margin = params["grid_margin"].as<double>();
	prob_data.grid_resolution = params["grid_resolution"].as<double>();
	prob_data.visibility_margin = params["visibility_margin"].as<double>();
	prob_data.distance_to_obs_margin = params["distance_to_obs_margin"].as<double>();

	prob_data.max_sim_time = params["max_time"].as<int>();
	prob_data.weight_goal = params["weight_goal"].as<double>();
	prob_data.weight_smoothness = params["weight_smoothness"].as<double>();
	prob_data.weight_goal_og = params["weight_goal"].as<double>();
	prob_data.weight_smoothness_og = params["weight_smoothness"].as<double>();
	
	prob_data.delta_aggressive = params["delta_aggressive"].as<double>();
	prob_data.delta_static_obs = params["delta_static_obs"].as<double>();
	prob_data.delta_drone = params["delta_drone"].as<double>();
	prob_data.delta_vel = params["delta_vel"].as<double>();
	prob_data.delta_acc = params["delta_acc"].as<double>();
	prob_data.delta_jerk = params["delta_jerk"].as<double>();
	prob_data.delta_snap = params["delta_snap"].as<double>();
	prob_data.delta_ineq = params["delta_ineq"].as<double>();
	prob_data.delta_sfc = params["delta_sfc"].as<double>();

	prob_data.rho_static_obs_max = params["rho_static_obs_max"].as<double>();
	prob_data.rho_drone_max = params["rho_drone_max"].as<double>();
	prob_data.rho_vel_max = params["rho_vel_max"].as<double>();
	prob_data.rho_acc_max = params["rho_acc_max"].as<double>();
	prob_data.rho_jerk_max = params["rho_jerk_max"].as<double>();
	prob_data.rho_snap_max = params["rho_snap_max"].as<double>();
	prob_data.rho_ineq_max = params["rho_ineq_max"].as<double>();
	prob_data.rho_sfc_max = params["rho_sfc_max"].as<double>();

	prob_data.axis_wise = false;//params["axis_wise"].as<bool>();
	prob_data.jerk_snap_constraints = params["jerk_snap_constraints"].as<bool>();
	if(!prob_data.axis_wise){
		prob_data.vel_max = params["vel_max"].as<double>();
		prob_data.acc_max = params["acc_max"].as<double>();
		prob_data.jerk_max = params["jerk_max"].as<double>();
		prob_data.snap_max = params["snap_max"].as<double>();
	}
	else{
		prob_data.vel_max = params["vel_max"].as<double>()/sqrt(3);
		prob_data.acc_max = params["acc_max"].as<double>()/sqrt(3);
		prob_data.jerk_max = params["jerk_max"].as<double>()/sqrt(3);
		prob_data.snap_max = params["snap_max"].as<double>()/sqrt(3);
	}
	prob_data.x_min = std::max(prob_data.world_dimension[0][0], _x_lim[0]) + params["a_drone"].as<double>();
	prob_data.y_min = std::max(prob_data.world_dimension[1][0], _y_lim[0]) + params["b_drone"].as<double>();
	prob_data.z_min = std::max(prob_data.world_dimension[2][0], _z_lim[0]) + params["a_drone"].as<double>();
	
	prob_data.x_max = std::min(prob_data.world_dimension[0][1], _x_lim[1]) - params["a_drone"].as<double>();
	prob_data.y_max = std::min(prob_data.world_dimension[1][1], _y_lim[1]) - params["b_drone"].as<double>();
	prob_data.z_max = std::min(prob_data.world_dimension[2][1], _z_lim[1]) - params["a_drone"].as<double>();

	prob_data.thresold = params["thresold"].as<double>();

	prob_data.prox_agent = params["prox_agent"].as<double>();
	prob_data.prox_obs = params["prox_obs"].as<double>();
	prob_data.buffer = params["buffer"].as<double>();
	
	prob_data.gamma = params["gamma"].as<double>();
	prob_data.gravity = params["gravity"].as<double>();	
	prob_data.use_thrust_values = params["use_thrust_values"].as<bool>();
	prob_data.f_min = params["f_min"].as<double>() * prob_data.gravity;
	prob_data.f_max = params["f_max"].as<double>() * prob_data.gravity;

	prob_data.dt = prob_data.t_plan / prob_data.num;
	

	// prob_data.anchor_points = params["anchor_points"].as< std :: vector<std::vector<double>>>();
	prob_data.not_converged_count = 0;
	prob_data.waypoint_index = 0;
	prob_data.anchor_index = 0;
	
	// @ Initial State
	prob_data.vx_init = 0.0;
	prob_data.vy_init = 0.0;
	prob_data.vz_init = 0.0;

	prob_data.az_init = 0.0;
	prob_data.ay_init = 0.0;
	prob_data.ax_init = 0.0;

	prob_data.x_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.x_goal;
	prob_data.y_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.y_goal;
	prob_data.z_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.z_goal;
	
	// @ Compute Bernstein P, Pdot, Pddot,... matrix
	Eigen :: ArrayXd tot_time = Eigen :: ArrayXd(prob_data.num);
	Eigen :: ArrayXd tot_time_up = Eigen :: ArrayXd(prob_data.num_up);
	
	tot_time.setLinSpaced(prob_data.num, 0.0, prob_data.t_plan);
	tot_time_up.setLinSpaced(prob_data.num_up, 0.0, prob_data.t_plan);

	five_var PPP =  computeBernstein(tot_time, prob_data.t_plan, prob_data.num, prob_data.pieces, prob_data.degree);
	prob_data.nvar = PPP.a.cols();
	prob_data.P = PPP.a;
	prob_data.Pdot = PPP.b;
	prob_data.Pddot = PPP.c;
	prob_data.Pdddot = PPP.d;
	prob_data.Pddddot = PPP.e;

	five_var PPP_up =  computeBernstein(tot_time_up, prob_data.t_plan, prob_data.num, prob_data.pieces, prob_data.degree);
	prob_data.P_up = PPP_up.a;
	prob_data.Pdot_up = PPP_up.b;
	prob_data.Pddot_up = PPP_up.c;
	prob_data.Pdddot_up = PPP_up.d;
	prob_data.Pddddot_up = PPP_up.e;

	// @ Position Constraints
	prob_data.A_ineq =  stack(prob_data.P, -prob_data.P, 'v');

	prob_data.b_x_ineq =  stack(prob_data.x_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), -prob_data.x_min * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
	prob_data.b_y_ineq =  stack(prob_data.y_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), -prob_data.y_min * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
	prob_data.b_z_ineq =  stack(prob_data.z_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), -prob_data.z_min * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');

	
	// @ Initial Conditions
	prob_data.A_init =  stack(prob_data.P.row(0),  stack(prob_data.Pdot.row(0), prob_data.Pddot.row(0), 'v'), 'v');
	for(int i = 1; i < prob_data.pieces; i++){
		int index_temp_1 = i*prob_data.num/prob_data.pieces-1;
		int index_temp_2 = i*prob_data.num/prob_data.pieces;

		if(i == 1){
			prob_data.A_conti =  stack( stack(-prob_data.P.row(index_temp_1)+prob_data.P.row(index_temp_2), -prob_data.Pdot.row(index_temp_1)+prob_data.Pdot.row(index_temp_2), 'v'), 
												stack(-prob_data.Pddot.row(index_temp_1)+prob_data.Pddot.row(index_temp_2),-prob_data.Pdddot.row(index_temp_1)+prob_data.Pdddot.row(index_temp_2), 'v'), 'v');
		}
		else{
			Eigen :: ArrayXXd temp =  stack( stack(-prob_data.P.row(index_temp_1)+prob_data.P.row(index_temp_2), -prob_data.Pdot.row(index_temp_1)+prob_data.Pdot.row(index_temp_2), 'v'), 
												stack(-prob_data.Pddot.row(index_temp_1)+prob_data.Pddot.row(index_temp_2),-prob_data.Pdddot.row(index_temp_1)+prob_data.Pdddot.row(index_temp_2), 'v'), 'v');
			prob_data.A_conti = stack(prob_data.A_conti, temp, 'v');
		}
	}
	if(prob_data.pieces > 1){
		prob_data.b_x_conti = Eigen :: ArrayXXd :: Zero(prob_data.A_conti.rows(), 1);
		prob_data.b_y_conti = Eigen :: ArrayXXd :: Zero(prob_data.A_conti.rows(), 1);
		prob_data.b_z_conti = Eigen :: ArrayXXd :: Zero(prob_data.A_conti.rows(), 1);
	}

	prob_data.b_x_init = Eigen :: ArrayXXd(prob_data.A_init.rows(), 1);
	prob_data.b_y_init = Eigen :: ArrayXXd(prob_data.A_init.rows(), 1);
	prob_data.b_z_init = Eigen :: ArrayXXd(prob_data.A_init.rows(), 1);

	prob_data.b_x_init << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
	prob_data.b_y_init << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;
	prob_data.b_z_init << prob_data.z_init, prob_data.vz_init, prob_data.az_init;

	
	// @ SFC
	prob_data.alpha_sfc = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.beta_sfc = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * M_PI_2;
	prob_data.d_sfc = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	

	// @ Static Obstacle Avoidance Constraints
	if(prob_data.num_static_obs!=0){
		prob_data.A_static_obs = prob_data.P;
		for(int i = 0; i < prob_data.num_static_obs - 1; i++) prob_data.A_static_obs =  stack(prob_data.A_static_obs, prob_data.P, 'v');
		
		prob_data.alpha_static_obs = Eigen :: ArrayXXd :: Zero(prob_data.num_static_obs, prob_data.num);
		prob_data.d_static_obs = Eigen :: ArrayXXd :: Ones(prob_data.num_static_obs, prob_data.num);
		prob_data.d_static_obs_old = prob_data.d_static_obs;

		prob_data.x_static_obs = (Eigen :: ArrayXXd :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * x_static_obs; 
		prob_data.y_static_obs = (Eigen :: ArrayXXd :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * y_static_obs;
		prob_data.z_static_obs = (Eigen :: ArrayXXd :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * z_static_obs;

		prob_data.a_static_obs = (Eigen :: ArrayXXd :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * a_static_obs + prob_data.buffer + params["a_drone"].as<double>();
		prob_data.b_static_obs = (Eigen :: ArrayXXd :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * b_static_obs + prob_data.buffer + params["b_drone"].as<double>();
		prob_data.c_static_obs = (Eigen :: ArrayXXd :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * c_static_obs + prob_data.buffer + params["c_drone"].as<double>();

		prob_data.x_static_obs_og = prob_data.x_static_obs;
		prob_data.y_static_obs_og = prob_data.y_static_obs;
		prob_data.z_static_obs_og = prob_data.z_static_obs;

		prob_data.a_static_obs_og = prob_data.a_static_obs;
		prob_data.b_static_obs_og = prob_data.b_static_obs;
		prob_data.c_static_obs_og = prob_data.c_static_obs;
	}
	
	prob_data.lx_drone = params["a_drone"].as<double>();
	prob_data.ly_drone = params["b_drone"].as<double>();
	prob_data.lz_drone = params["c_drone"].as<double>();
	
	// @ Inter-agent Collision Avoidance Constraints
	if(prob_data.num_drone!=0){

		prob_data.a_drone = Eigen :: ArrayXXd :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.lx_drone + prob_data.buffer);
		prob_data.b_drone = Eigen :: ArrayXXd :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.ly_drone + prob_data.buffer);
		prob_data.c_drone = Eigen :: ArrayXXd :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.lz_drone + prob_data.buffer);
	
		prob_data.alpha_drone = Eigen :: ArrayXXd :: Zero(prob_data.num_drone, prob_data.num);
		prob_data.beta_drone = Eigen :: ArrayXXd :: Ones(prob_data.num_drone, prob_data.num) * M_PI_2;
		prob_data.d_drone = Eigen :: ArrayXXd :: Ones(prob_data.num_drone, prob_data.num);
		prob_data.d_drone_old = prob_data.d_drone;

		prob_data.A_drone = prob_data.P;    
		for(int i = 0; i < prob_data.num_drone - 1; i++) prob_data.A_drone =  stack(prob_data.A_drone, prob_data.P, 'v');
	}
	else{
		ROS_WARN_STREAM("Only one drone in environment");
	}

	if(!prob_data.axis_wise){
		// @ Velocity Constraints
		prob_data.alpha_vel = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
		prob_data.beta_vel = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * M_PI_2 * 0;
		prob_data.d_vel = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * prob_data.vel_max ;
		
		// @ Acceleration Constraints
		prob_data.alpha_acc = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
		prob_data.beta_acc = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * M_PI_2 * 0;
		prob_data.d_acc = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * prob_data.acc_max ;
		
		
		// @ Jerk Constraints
		prob_data.alpha_jerk = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
		prob_data.beta_jerk = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * M_PI_2 * 0;
		prob_data.d_jerk = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * prob_data.jerk_max;
		
		
		// @ Snap Constraints
		prob_data.alpha_snap = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
		prob_data.beta_snap = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * M_PI_2 * 0;
		prob_data.d_snap = Eigen :: ArrayXXd :: Ones(prob_data.num, 1) * prob_data.snap_max * 0;
	}
	else{
		// @ Velocity Constraints
		prob_data.A_v_ineq =  stack(prob_data.Pdot, -prob_data.Pdot, 'v');

		prob_data.b_vx_ineq =  stack(prob_data.vel_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
		prob_data.b_vy_ineq =  stack(prob_data.vel_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
		prob_data.b_vz_ineq =  stack(prob_data.vel_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');

		// @ Acceleration Constraints
		prob_data.A_a_ineq =  stack(prob_data.Pddot, -prob_data.Pddot, 'v');

		if(!prob_data.use_thrust_values && prob_data.world == 2){
			prob_data.b_ax_ineq =  stack(prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
			prob_data.b_ay_ineq =  stack(prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
			prob_data.b_az_ineq =  stack(prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
		}
		else{

			/** 
			 * 2x^2 + (x+g)^2 = (1.5g)^2
			 * 2x^2 + x^2 + 2xg + g^2-(1.5g)^2=0
			 * 3x^2 + 2xg + (g^2-(1.5g)^2)
			 * **/

			prob_data.acc_max = (-(2*prob_data.gravity) + sqrt(pow(2*prob_data.gravity,2) - 4*3*(pow(prob_data.gravity,2) - pow(1.5*prob_data.gravity,2))))/6.0; 
			double z_acc_min = prob_data.f_min - prob_data.gravity;
			
			prob_data.b_ax_ineq =  stack(prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
			prob_data.b_ay_ineq =  stack(prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
			prob_data.b_az_ineq =  stack(prob_data.acc_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), -z_acc_min * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
			
		}
		// @ Jerk Constraints
		prob_data.A_j_ineq =  stack(prob_data.Pdddot, -prob_data.Pdddot, 'v');

		prob_data.b_jx_ineq =  stack(prob_data.jerk_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
		prob_data.b_jy_ineq =  stack(prob_data.jerk_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
		prob_data.b_jz_ineq =  stack(prob_data.jerk_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');

		// @ Snap Constraints
		prob_data.A_s_ineq =  stack(prob_data.Pddddot, -prob_data.Pddddot, 'v');

		prob_data.b_sx_ineq =  stack(prob_data.snap_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
		prob_data.b_sy_ineq =  stack(prob_data.snap_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
		prob_data.b_sz_ineq =  stack(prob_data.snap_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXd :: Ones(prob_data.num, 1), 'v');
	}
	
	// @ Constant Costs
	prob_data.cost_ineq = prob_data.A_ineq.transpose().matrix() * prob_data.A_ineq.matrix();
	if(prob_data.params["order_smoothness"].as<int>() == 4)
		prob_data.cost_smoothness = prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix();
	else if(prob_data.params["order_smoothness"].as<int>() == 3)
		prob_data.cost_smoothness = prob_data.Pdddot.transpose().matrix() * prob_data.Pdddot.matrix();
	else if(prob_data.params["order_smoothness"].as<int>() == 2)
		prob_data.cost_smoothness = prob_data.Pddot.transpose().matrix() * prob_data.Pddot.matrix();
	else	
		ROS_ERROR_STREAM("Invalid order_smoothness value");

	
	prob_data.cost_goal = prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.P.bottomRows(prob_data.kappa).matrix();    
	prob_data.cost_sfc = prob_data.P.transpose().matrix() * prob_data.P.matrix();
	prob_data.I = Eigen :: ArrayXXd(prob_data.nvar + prob_data.A_init.rows() + prob_data.A_conti.rows(), prob_data.nvar + prob_data.A_init.rows() + prob_data.A_conti.rows());
	prob_data.I.matrix().setIdentity();

	if(!prob_data.axis_wise){
		prob_data.cost_vel = prob_data.Pdot.transpose().matrix() * prob_data.Pdot.matrix();
		prob_data.cost_acc = prob_data.Pddot.transpose().matrix() * prob_data.Pddot.matrix();
		prob_data.cost_jerk = prob_data.Pdddot.transpose().matrix() * prob_data.Pdddot.matrix();
		prob_data.cost_snap = prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix();
	}
	else{
		prob_data.cost_vel = prob_data.A_v_ineq.transpose().matrix() * prob_data.A_v_ineq.matrix();
		prob_data.cost_acc = prob_data.A_a_ineq.transpose().matrix() * prob_data.A_a_ineq.matrix();
		prob_data.cost_jerk = prob_data.A_j_ineq.transpose().matrix() * prob_data.A_j_ineq.matrix();
		prob_data.cost_snap = prob_data.A_s_ineq.transpose().matrix() * prob_data.A_s_ineq.matrix();
	}
	

	prob_data.x = prob_data.x_init * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);
	prob_data.x_anchor =  prob_data.anchor_points[0][0] * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);//prob_data.x_init * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);
	prob_data.xdot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.xddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.xdddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.xddddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);

	prob_data.y = prob_data.y_init * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);
	prob_data.y_anchor =  prob_data.anchor_points[0][1] * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);//prob_data.y_init * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);
	prob_data.ydot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.yddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.ydddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.yddddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);

	prob_data.z = prob_data.z_init * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);
	prob_data.z_anchor = prob_data.anchor_points[0][2] * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);//prob_data.z_init * Eigen :: ArrayXXd :: Ones(prob_data.num, 1);
	prob_data.zdot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.zddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.zdddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);
	prob_data.zddddot = Eigen :: ArrayXXd :: Zero(prob_data.num, 1);

	prob_data.num_drone = 0;
	prob_data.num_static_obs = 0;

	if(prob_data.world == 2)
			getGrid2DPath(prob_data, VERBOSE);
	else if(prob_data.world == 3)
			getGrid3DPath(prob_data, VERBOSE);
	else 
		ROS_INFO_STREAM("Not using grid planner");
}

void deployAgent(probData &prob_data, int VERBOSE){
	std::chrono::high_resolution_clock::time_point start, end;
	std :: chrono :: duration<double, std::milli> total_time;

	// Initialize paramters and build matrices
	if(prob_data.mpc_step == 0){
		// ROS_INFO_STREAM("Initializing Drone " << prob_data.id_badge);
			initializeOptimizer(prob_data, VERBOSE);	
	}

	// Generate grid path, select anchor and intermediate goal
	if(prob_data.path_replan){
		start = std :: chrono :: high_resolution_clock::now(); 
		prob_data.anchor_index = 0;
		if(prob_data.world == 2)
				getGrid2DPath(prob_data, VERBOSE);
		else if(prob_data.world == 3)
				getGrid3DPath(prob_data, VERBOSE);
		else
			ROS_ERROR_STREAM("Cannot identify world");
		
		end = std :: chrono :: high_resolution_clock::now();
		total_time = (end - start);
		if(VERBOSE == 5)
			ROS_INFO_STREAM("Time take to compute grid path: " <<  total_time.count()/1000);
	}
	
	if(!prob_data.sfcc)
		assignAnchorPoints(prob_data, VERBOSE);
	
	setGoal(prob_data, VERBOSE);

	if(prob_data.sfcc){
		if(prob_data.world == 2)
			getConvexPolytope2D(prob_data, VERBOSE);
		else if(prob_data.world == 3)
			getConvexPolytope3D(prob_data, VERBOSE);		
		if(VERBOSE == 5)
			ROS_INFO_STREAM("ConvexPolytope constructed");
	}

	// @ Initialize alpha betas ds
	if(prob_data.mpc_step > 0){
			neigbhoringAgents(prob_data, VERBOSE);		
		start = std :: chrono :: high_resolution_clock::now();
		if(prob_data.world == 2)
				initAlpha(prob_data, VERBOSE);
		else if(prob_data.world == 3)
				initAlphaBeta(prob_data, VERBOSE);
		else
			ROS_ERROR_STREAM("Cannot identify world");
		end = std :: chrono :: high_resolution_clock::now();
		total_time = (end - start);
		if(VERBOSE == 5)
			ROS_INFO_STREAM("Time take to compute alpha beta: " <<  total_time.count()/1000);
	}
	

	// @ Solve xyz
	start = std :: chrono :: high_resolution_clock::now(); 
	if(prob_data.world == 2)
			computeXY(prob_data, VERBOSE);
	else if(prob_data.world == 3)
			computeXYZ(prob_data, VERBOSE);

	end = std :: chrono :: high_resolution_clock::now();
	total_time = (end - start);
	if(VERBOSE == 5)
		ROS_INFO_STREAM("Time take to compute trajectory: " <<  total_time.count()/1000);

	prob_data.smoothness.push_back(sqrt(pow(prob_data.ax_init, 2) + pow(prob_data.ay_init, 2) + pow(prob_data.az_init, 2)));
	prob_data.arc_length.push_back(sqrt(pow(prob_data.x_init - prob_data.x(1), 2) + pow(prob_data.y_init - prob_data.y(1), 2) + pow(prob_data.z_init - prob_data.z(1), 2)));		

	prob_data.dist_to_goal = sqrt(pow(prob_data.x_init - prob_data.xf_goal, 2) + pow(prob_data.y_init - prob_data.yf_goal, 2) + pow(prob_data.z_init - prob_data.zf_goal, 2));
	if(prob_data.dist_to_goal <= 1.0)
		prob_data.weight_goal = 100000;
	else
		prob_data.weight_goal = prob_data.weight_goal_og;
	if(VERBOSE == 1)
			checkResiduals(prob_data, VERBOSE);
}

