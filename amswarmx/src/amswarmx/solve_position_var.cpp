#include "amswarmx/solve_position_var.hpp"

void computeXY(probData &prob_data, int VERBOSE)
{
							
	Eigen :: ArrayXXd cost_x, cost_x_inv, cost_y, cost_y_inv;
	Eigen :: ArrayXXd objective_x, objective_y, lincost_x, lincost_y;
	Eigen :: ArrayXXd cost_drone, cost_static_obs;
	Eigen :: ArrayXXd temp_x_static_obs, temp_y_static_obs;
	Eigen :: ArrayXXd temp_x_drone, temp_y_drone;
	Eigen :: ArrayXXd temp_x_sfc, temp_y_sfc, temp_A_x_cp, temp_A_y_cp;
	Eigen :: ArrayXXd sol_x, sol_y, primal_sol_x, primal_sol_y;
	Eigen :: ArrayXXd res_x_static_obs, res_y_static_obs,
						res_x_drone, res_y_drone,
						res_x_vel, res_y_vel,
						res_x_acc, res_y_acc,
						res_x_jerk, res_y_jerk,
						res_x_snap, res_y_snap,
						res_x_ineq, res_y_ineq,
						res_x_sfc, res_y_sfc;
	
	// @ Set Initial Conidtions for next MPC step
	prob_data.b_x_init << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
	prob_data.b_y_init << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;

	if(prob_data.pieces == 1){
		prob_data.A_eq = prob_data.A_init;
		prob_data.b_x_eq = prob_data.b_x_init;
		prob_data.b_y_eq = prob_data.b_y_init;
	}
	else{
		prob_data.A_eq = stack(prob_data.A_init, prob_data.A_conti, 'v');
		prob_data.b_x_eq = stack(prob_data.b_x_init, prob_data.b_x_conti, 'v');
		prob_data.b_y_eq = stack(prob_data.b_y_init, prob_data.b_y_conti, 'v');
	}

	float thresold = prob_data.thresold;

	prob_data.rho_static_obs = 1.0;
	prob_data.rho_drone = 1.0;
	prob_data.rho_vel = 1.0;
	prob_data.rho_acc = 1.0;
	prob_data.rho_jerk = 1.0;
	prob_data.rho_snap = 1.0;
	prob_data.rho_ineq = 1.0;
	prob_data.rho_sfc = 1.0;
	
	// @ Lagrange Multiplier
	prob_data.lamda_x = Eigen :: ArrayXXd :: Ones(prob_data.nvar, 1) *0;
	prob_data.lamda_y = Eigen :: ArrayXXd :: Ones(prob_data.nvar, 1) *0;

	// @ Position Constraints
	prob_data.s_x_ineq = Eigen :: ArrayXXd :: Ones(2*prob_data.num, 1) *0;
	prob_data.s_y_ineq = Eigen :: ArrayXXd :: Ones(2*prob_data.num, 1) *0;

	if(prob_data.sfcc){
		prob_data.s_cp = Eigen :: ArrayXXd :: Ones(prob_data.b_cp.rows(), 1) *0;
	}
	if(prob_data.num_drone!=0)cost_drone = prob_data.A_drone.transpose().matrix() * prob_data.A_drone.matrix();
	if(prob_data.num_static_obs!=0)cost_static_obs = prob_data.A_static_obs.transpose().matrix() * prob_data.A_static_obs.matrix();
	
	
	int break_flag;
	for(int i = 0; i < prob_data.max_iter; i++){

		prob_data.prev_x = prob_data.x;
		prob_data.prev_y = prob_data.y;

		break_flag = 0;

		// x - Position, velocity and acceleration constraints 
		prob_data.b_x_vel = prob_data.d_vel * cos(prob_data.alpha_vel);
		prob_data.b_x_acc = prob_data.d_acc * cos(prob_data.alpha_acc);
		prob_data.B_x_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_x_ineq - prob_data.s_x_ineq).matrix();
		objective_x = prob_data.weight_goal * prob_data.cost_goal 
				+ prob_data.weight_smoothness * prob_data.cost_smoothness 
				+ prob_data.rho_vel * prob_data.cost_vel 
				+ prob_data.rho_acc * prob_data.cost_acc 
				+ prob_data.rho_ineq * prob_data.cost_ineq;
		lincost_x = -prob_data.lamda_x
					-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.x_ref.matrix()).array()
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_x_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_x_acc.matrix()).array()
					-prob_data.rho_ineq * prob_data.B_x_ineq;

		// x - Jerk and snap constraints
		if(prob_data.jerk_snap_constraints){
			prob_data.b_x_jerk = prob_data.d_jerk * cos(prob_data.alpha_jerk);
			prob_data.b_x_snap = prob_data.d_snap * cos(prob_data.alpha_snap);
			objective_x += prob_data.rho_jerk * prob_data.cost_jerk
				+ prob_data.rho_snap * prob_data.cost_snap;
			lincost_x += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_x_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_x_snap.matrix()).array();
		}			

		// x - cylindrical static obstacles constraints
		if(prob_data.num_static_obs!=0){
			temp_x_static_obs =  prob_data.x_static_obs + prob_data.d_static_obs * cos(prob_data.alpha_static_obs) * prob_data.a_static_obs;
			prob_data.b_x_static_obs =  reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
			objective_x += prob_data.rho_static_obs * cost_static_obs;
			lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
		}

		// x - ellipsoidal inter-agent constraints 
		if(prob_data.num_drone!=0){
			
			temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * prob_data.a_drone;
			prob_data.b_x_drone =  reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
			objective_x += prob_data.rho_drone * cost_drone;
			lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
		}

		// x - safe flight corridor constraints
		if(!prob_data.sfcc){
			objective_x += prob_data.rho_sfc * prob_data.cost_sfc;
			temp_x_sfc =  (prob_data.x_anchor + prob_data.d_sfc * cos(prob_data.alpha_sfc));
			lincost_x += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * temp_x_sfc.matrix()).array();
		}
		else if(prob_data.sfcc ){
			temp_A_x_cp = prob_data.A_x_cp.matrix() * prob_data.P.matrix();
			objective_x += prob_data.rho_sfc * (temp_A_x_cp.transpose().matrix() * temp_A_x_cp.matrix()).array();
			prob_data.B_x_cp = temp_A_x_cp.transpose().matrix()*(-(prob_data.A_y_cp.matrix() * prob_data.y.matrix()).array() + prob_data.b_cp - prob_data.s_cp).matrix(); 
			lincost_x += -prob_data.rho_sfc * prob_data.B_x_cp;
		}

		// @x - Solve set of linear equations
		cost_x =  stack(stack(objective_x, prob_data.A_eq.transpose(), 'h'), 
							stack(prob_data.A_eq, Eigen::ArrayXXd::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
		cost_x_inv = (cost_x.matrix()).householderQr().solve(prob_data.I.matrix());
		sol_x = cost_x_inv.matrix() * stack(-lincost_x, prob_data.b_x_eq, 'v').matrix();
		primal_sol_x = sol_x.topRows(prob_data.nvar);
		prob_data.x = prob_data.P.matrix() * primal_sol_x.matrix();
		prob_data.xdot = prob_data.Pdot.matrix() * primal_sol_x.matrix();
		prob_data.xddot = prob_data.Pddot.matrix() * primal_sol_x.matrix();
		prob_data.xdddot = prob_data.Pdddot.matrix() * primal_sol_x.matrix();
		prob_data.xddddot = prob_data.Pddddot.matrix() * primal_sol_x.matrix();
		prob_data.x_up = prob_data.P_up.matrix() * primal_sol_x.matrix();
		prob_data.xdot_up = prob_data.Pdot_up.matrix() * primal_sol_x.matrix();
		prob_data.xddot_up = prob_data.Pddot_up.matrix() * primal_sol_x.matrix();


		// y - Position, velocity and acceleration constraints 
		prob_data.b_y_vel = prob_data.d_vel * sin(prob_data.alpha_vel);
		prob_data.b_y_acc = prob_data.d_acc * sin(prob_data.alpha_acc);
		prob_data.B_y_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_y_ineq - prob_data.s_y_ineq).matrix();
		objective_y = prob_data.weight_goal * prob_data.cost_goal 
				+ prob_data.weight_smoothness * prob_data.cost_smoothness 
				+ prob_data.rho_vel * prob_data.cost_vel 
				+ prob_data.rho_acc * prob_data.cost_acc 
				+ prob_data.rho_ineq * prob_data.cost_ineq;
		lincost_y = -prob_data.lamda_y
					-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.y_ref.matrix()).array()
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_y_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_y_acc.matrix()).array()
					-prob_data.rho_ineq * prob_data.B_y_ineq;
		
		// y - Jerk and snap constraints
		if(prob_data.jerk_snap_constraints){
			prob_data.b_y_jerk = prob_data.d_jerk * sin(prob_data.alpha_jerk);
			prob_data.b_y_snap = prob_data.d_snap * sin(prob_data.alpha_snap);
			objective_y += prob_data.rho_jerk * prob_data.cost_jerk
				+ prob_data.rho_snap * prob_data.cost_snap;
			lincost_y += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_y_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_y_snap.matrix()).array();
		}

		// y - cylindrical static obstacles constraints
		if(prob_data.num_static_obs!=0){
			temp_y_static_obs =  prob_data.y_static_obs + prob_data.d_static_obs * sin(prob_data.alpha_static_obs) * prob_data.b_static_obs;
			prob_data.b_y_static_obs =  reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);	
			objective_y += prob_data.rho_static_obs * cost_static_obs;
			lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();
		}

		// y - ellipsoidal inter-agent constraints 
		if(prob_data.num_drone!=0){
			temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * prob_data.b_drone;
			prob_data.b_y_drone =  reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
			objective_y += prob_data.rho_drone * cost_drone;
			lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();
		}	

		// y - safe flight corridor constraints
		if(!prob_data.sfcc){
			objective_y += prob_data.rho_sfc * prob_data.cost_sfc;
			temp_y_sfc =  (prob_data.y_anchor + prob_data.d_sfc * sin(prob_data.alpha_sfc));			
			lincost_y += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * temp_y_sfc.matrix()).array();
		}
		else if(prob_data.sfcc ){
			temp_A_y_cp = prob_data.A_y_cp.matrix() * prob_data.P.matrix();
			objective_y += prob_data.rho_sfc * (temp_A_y_cp.transpose().matrix() * temp_A_y_cp.matrix()).array();
			prob_data.B_y_cp = temp_A_y_cp.transpose().matrix()*(-(prob_data.A_x_cp.matrix() * prob_data.x.matrix()).array() + prob_data.b_cp - prob_data.s_cp).matrix(); 
			lincost_y += -prob_data.rho_sfc * prob_data.B_y_cp; 
		}

		// @y - Solve set of linear equations
		cost_y =  stack(stack(objective_y, prob_data.A_eq.transpose(), 'h'), 
							stack(prob_data.A_eq, Eigen::ArrayXXd::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
		cost_y_inv = (cost_y.matrix()).householderQr().solve(prob_data.I.matrix());
		sol_y = cost_y_inv.matrix() * stack(-lincost_y, prob_data.b_y_eq, 'v').matrix(); 
		primal_sol_y = sol_y.topRows(prob_data.nvar);
		prob_data.y = prob_data.P.matrix() * primal_sol_y.matrix();
		prob_data.ydot = prob_data.Pdot.matrix() * primal_sol_y.matrix();
		prob_data.yddot = prob_data.Pddot.matrix() * primal_sol_y.matrix();
		prob_data.ydddot = prob_data.Pdddot.matrix() * primal_sol_y.matrix();
		prob_data.yddddot = prob_data.Pddddot.matrix() * primal_sol_y.matrix();	
		prob_data.y_up = prob_data.P_up.matrix() * primal_sol_y.matrix();
		prob_data.ydot_up = prob_data.Pdot_up.matrix() * primal_sol_y.matrix();
		prob_data.yddot_up = prob_data.Pddot_up.matrix() * primal_sol_y.matrix();
		
		
		// @Solve for polar and d variables
		initAlpha(prob_data, VERBOSE);
		
		// @Update residuals and Lagrange multiplier
		// position, velocity, acceleration constraint residuals
		prob_data.s_x_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_x_ineq).max(0.0);
		res_x_ineq = (prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_x_ineq + prob_data.s_x_ineq;
		res_x_vel = prob_data.xdot - prob_data.d_vel * cos(prob_data.alpha_vel);
		res_x_acc = prob_data.xddot - prob_data.d_acc * cos(prob_data.alpha_acc);
		prob_data.lamda_x = prob_data.lamda_x 
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_x_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_x_acc.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_x_ineq.matrix()).array();
		prob_data.res_x_ineq_norm = res_x_ineq.matrix().norm();
		prob_data.res_x_vel_norm = res_x_vel.matrix().norm();
		prob_data.res_x_acc_norm = res_x_acc.matrix().norm();


		prob_data.s_y_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_y_ineq).max(0.0);
		res_y_ineq = (prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_y_ineq + prob_data.s_y_ineq;
		res_y_vel = prob_data.ydot - prob_data.d_vel * sin(prob_data.alpha_vel);
		res_y_acc = prob_data.yddot - prob_data.d_acc * sin(prob_data.alpha_acc);
		prob_data.lamda_y = prob_data.lamda_y 
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_y_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_y_acc.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_y_ineq.matrix()).array();
		prob_data.res_y_ineq_norm = res_y_ineq.matrix().norm();
		prob_data.res_y_vel_norm = res_y_vel.matrix().norm();
		prob_data.res_y_acc_norm = res_y_acc.matrix().norm();

		prob_data.res_z_ineq_norm = 0.0;
		prob_data.res_z_vel_norm = 0.0;
		prob_data.res_z_acc_norm = 0.0;

		if((prob_data.res_x_ineq_norm > thresold || prob_data.res_y_ineq_norm > thresold) && prob_data.rho_ineq){
				prob_data.rho_ineq *= prob_data.delta_ineq;
				if(prob_data.rho_ineq > prob_data.rho_ineq_max) prob_data.rho_ineq = prob_data.rho_ineq_max;
		}
		else{
			prob_data.rho_ineq /= prob_data.delta_ineq;
			if(prob_data.rho_ineq < 1.0) prob_data.rho_ineq = 1.0;
			break_flag++;
		}
		if(prob_data.res_x_vel_norm > thresold || prob_data.res_y_vel_norm > thresold){
				prob_data.rho_vel *= prob_data.delta_vel;
				if(prob_data.rho_vel > prob_data.rho_vel_max) prob_data.rho_vel = prob_data.rho_vel_max;
		}
		else{
			prob_data.rho_vel /= prob_data.delta_vel;
			if(prob_data.rho_vel < 1.0) prob_data.rho_vel = 1.0;
			break_flag++;
		}
		if(prob_data.res_x_acc_norm > thresold || prob_data.res_y_acc_norm > thresold){
				prob_data.rho_acc *= prob_data.delta_acc;
				if(prob_data.rho_acc > prob_data.rho_acc_max) prob_data.rho_acc = prob_data.rho_acc_max;
		}
		else{
			prob_data.rho_acc /= prob_data.delta_acc;
			if(prob_data.rho_acc < 1.0) prob_data.rho_acc = 1.0;
			break_flag++;
		}

		// safe flight corridor constraints residulas
		if(!prob_data.sfcc){
			res_x_sfc = prob_data.x - prob_data.x_anchor - prob_data.d_sfc * cos(prob_data.alpha_sfc);
			prob_data.lamda_x += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * res_x_sfc.matrix()).array();
			prob_data.res_x_sfc_norm = res_x_sfc.matrix().norm();

			res_y_sfc = prob_data.y - prob_data.y_anchor - prob_data.d_sfc * sin(prob_data.alpha_sfc);
			prob_data.lamda_y += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * res_y_sfc.matrix()).array();
			prob_data.res_y_sfc_norm = res_y_sfc.matrix().norm();

			prob_data.res_z_sfc_norm = 0.0;

			if(prob_data.res_x_sfc_norm > thresold || prob_data.res_y_sfc_norm > thresold){
				prob_data.rho_sfc *= prob_data.delta_sfc;
				if(prob_data.rho_sfc > prob_data.rho_sfc_max) prob_data.rho_sfc = prob_data.rho_sfc_max;
			}
			else{
				prob_data.rho_sfc /= prob_data.delta_sfc;
				if(prob_data.rho_sfc < 1.0) prob_data.rho_sfc = 1.0;
				break_flag++;
			}
		}
		else if(prob_data.sfcc ){
			
			prob_data.s_cp = ((-(prob_data.A_x_cp.matrix() * prob_data.x.matrix() + prob_data.A_y_cp.matrix() * prob_data.y.matrix())).array() + prob_data.b_cp).max(0.0);

			res_x_sfc = (prob_data.A_x_cp.matrix() * prob_data.x.matrix() + prob_data.A_y_cp.matrix() * prob_data.y.matrix()).array() - prob_data.b_cp + prob_data.s_cp;
			prob_data.lamda_x += -prob_data.rho_sfc * (temp_A_x_cp.transpose().matrix() * res_x_sfc.matrix()).array();
			prob_data.res_x_sfc_norm = res_x_sfc.matrix().norm();

			res_y_sfc = (prob_data.A_x_cp.matrix() * prob_data.x.matrix() + prob_data.A_y_cp.matrix() * prob_data.y.matrix()).array() - prob_data.b_cp + prob_data.s_cp;
			prob_data.lamda_y += -prob_data.rho_sfc * (temp_A_y_cp.transpose().matrix() * res_y_sfc.matrix()).array();
			prob_data.res_y_sfc_norm = res_y_sfc.matrix().norm();

			prob_data.res_z_sfc_norm = 0.0;
			
			if(prob_data.res_x_sfc_norm > thresold || prob_data.res_y_sfc_norm > thresold){
				prob_data.rho_sfc *= prob_data.delta_sfc;
				if(prob_data.rho_sfc > prob_data.rho_sfc_max) prob_data.rho_sfc = prob_data.rho_sfc_max;
			}
			else{
				prob_data.rho_sfc /= prob_data.delta_sfc;
				if(prob_data.rho_sfc < 1.0) prob_data.rho_sfc = 1.0;
				break_flag++;
			}
		}

		// jerk and snap constraints residuals
		if(prob_data.jerk_snap_constraints){
			res_x_jerk = prob_data.xdddot - prob_data.d_jerk * cos(prob_data.alpha_jerk);
			res_x_snap = prob_data.xddddot - prob_data.d_snap * cos(prob_data.alpha_snap);
			prob_data.lamda_x += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_x_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_x_snap.matrix()).array();
			prob_data.res_x_jerk_norm = res_x_jerk.matrix().norm();
			prob_data.res_x_snap_norm = res_x_snap.matrix().norm();

			res_y_jerk = prob_data.ydddot - prob_data.d_jerk * sin(prob_data.alpha_jerk);
			res_y_snap = prob_data.yddddot - prob_data.d_snap * sin(prob_data.alpha_snap);
			prob_data.lamda_y += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_y_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_y_snap.matrix()).array();
			prob_data.res_y_jerk_norm = res_y_jerk.matrix().norm();
			prob_data.res_y_snap_norm = res_y_snap.matrix().norm();

			prob_data.res_z_jerk_norm = 0.0;
			prob_data.res_z_snap_norm = 0.0;

			if(prob_data.res_x_jerk_norm > thresold || prob_data.res_y_jerk_norm > thresold){;
				prob_data.rho_jerk *= prob_data.delta_jerk;
				if(prob_data.rho_jerk > prob_data.rho_jerk_max) prob_data.rho_jerk = prob_data.rho_jerk_max;
			}
			else{
				prob_data.rho_jerk /= prob_data.delta_jerk;
				if(prob_data.rho_jerk < 1.0) prob_data.rho_jerk = 1.0;
				break_flag++;
			}

			if(prob_data.res_x_snap_norm > thresold || prob_data.res_y_snap_norm > thresold){;
					prob_data.rho_snap *= prob_data.delta_snap;
					if(prob_data.rho_snap > prob_data.rho_snap_max) prob_data.rho_snap = prob_data.rho_snap_max;
			}
			else{
				prob_data.rho_snap /= prob_data.delta_snap;
				if(prob_data.rho_snap < 1.0) prob_data.rho_snap = 1.0;
				break_flag++;
			}
		}
		else{
			prob_data.res_x_jerk_norm = 0.0;
			prob_data.res_y_jerk_norm = 0.0;
			prob_data.res_z_jerk_norm = 0.0;

			prob_data.res_x_snap_norm = 0.0;
			prob_data.res_y_snap_norm = 0.0;
			prob_data.res_z_snap_norm = 0.0;

			break_flag += 2;
		}

		// cylindrical static obstacle avoidance constraint residuals
		if(prob_data.num_static_obs!=0){
			
			res_x_static_obs = reshape(((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_static_obs * prob_data.d_static_obs * cos(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
			prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_x_static_obs.matrix()).array();
			prob_data.res_x_static_obs_norm = res_x_static_obs.matrix().norm();

			res_y_static_obs = reshape(((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_static_obs * prob_data.d_static_obs * sin(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
			prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_y_static_obs.matrix()).array();
			prob_data.res_y_static_obs_norm = res_y_static_obs.matrix().norm();
		
			if(prob_data.res_x_static_obs_norm > thresold || prob_data.res_y_static_obs_norm > thresold){
				prob_data.rho_static_obs *= prob_data.delta_static_obs;
				if(prob_data.rho_static_obs > prob_data.rho_static_obs_max) prob_data.rho_static_obs = prob_data.rho_static_obs_max;
			}
			else{
				prob_data.rho_static_obs /= prob_data.delta_static_obs;
				if(prob_data.rho_static_obs < 1.0) prob_data.rho_static_obs = 1.0;
				break_flag++;
			}
		}
		else{
			prob_data.res_x_static_obs_norm = 0;
			prob_data.res_y_static_obs_norm = 0;
			break_flag++;
		}

		// ellipsoidal inter-agent constraints residuals
		if(prob_data.num_drone!=0){
			
			res_x_drone = reshape(((-prob_data.x_drone).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_drone * prob_data.d_drone * cos(prob_data.alpha_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
			prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_x_drone.matrix()).array();
			prob_data.res_x_drone_norm = res_x_drone.matrix().norm();

			res_y_drone = reshape(((-prob_data.y_drone).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_drone * prob_data.d_drone * sin(prob_data.alpha_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
			prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_y_drone.matrix()).array();
			prob_data.res_y_drone_norm = res_y_drone.matrix().norm();

			prob_data.res_z_drone_norm = 0;
			
			if(prob_data.res_x_drone_norm > thresold || prob_data.res_y_drone_norm > thresold){
				prob_data.rho_drone *= prob_data.delta_drone;
				if(prob_data.rho_drone > prob_data.rho_drone_max) prob_data.rho_drone = prob_data.rho_drone_max;
			}
			else{
				prob_data.rho_drone /= prob_data.delta_drone;
				if(prob_data.rho_drone < 1.0) prob_data.rho_drone = 1.0;
				break_flag++;
			}
		}
		else{
			prob_data.res_x_drone_norm = 0;
			prob_data.res_y_drone_norm = 0;
			prob_data.res_z_drone_norm = 0;
			break_flag++;
		}
		
		if(break_flag == 8){
			break;	
		}
	}
	
	if(break_flag != 8){
			prob_data.weight_goal *= prob_data.delta_aggressive;
			//prob_data.weight_smoothness *= prob_data.delta_aggressive;
		}
	else{
		prob_data.weight_goal = prob_data.weight_goal_og;
		prob_data.weight_smoothness = prob_data.weight_smoothness_og;

		if(prob_data.weight_goal > prob_data.weight_goal_og)
			prob_data.weight_goal = prob_data.weight_goal_og;

		if(prob_data.weight_goal > prob_data.weight_smoothness_og)
			prob_data.weight_smoothness = prob_data.weight_smoothness_og;
	}					
}


void computeXYZ(probData &prob_data, int VERBOSE)
{
							
	Eigen :: ArrayXXd cost_x, cost_y, cost_z, cost_x_inv, cost_y_inv, cost_z_inv;
	Eigen :: ArrayXXd objective_x, objective_y, objective_z, lincost_x, lincost_y, lincost_z;
	Eigen :: ArrayXXd cost_drone, cost_static_obs;
	Eigen :: ArrayXXd temp_A_x_cp, temp_A_y_cp, temp_A_z_cp; 
	Eigen :: ArrayXXd temp_x_static_obs, temp_y_static_obs;
	Eigen :: ArrayXXd temp_x_drone, temp_y_drone, temp_z_drone;
	Eigen :: ArrayXXd sol_x, sol_y, sol_z, primal_sol_x, primal_sol_y, primal_sol_z;
	Eigen :: ArrayXXd res_x_static_obs, res_y_static_obs,
						res_x_drone, res_y_drone, res_z_drone,
						res_x_vel, res_y_vel, res_z_vel,
						res_x_acc, res_y_acc, res_z_acc,
						res_x_jerk, res_y_jerk, res_z_jerk,
						res_x_snap, res_y_snap, res_z_snap,
						res_x_ineq, res_y_ineq, res_z_ineq,
						temp_x_sfc, temp_y_sfc, temp_z_sfc,
						res_x_sfc, res_y_sfc, res_z_sfc;
	
	// @ Set Initial Conidtions for next MPC step
	prob_data.b_x_init << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
	prob_data.b_y_init << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;
	prob_data.b_z_init << prob_data.z_init, prob_data.vz_init, prob_data.az_init;

	
	if(prob_data.pieces == 1){
		prob_data.A_eq = prob_data.A_init;
		prob_data.b_x_eq = prob_data.b_x_init;
		prob_data.b_y_eq = prob_data.b_y_init;
		prob_data.b_z_eq = prob_data.b_z_init;
	}
	else{
		prob_data.A_eq = stack(prob_data.A_init, prob_data.A_conti, 'v');
		prob_data.b_x_eq = stack(prob_data.b_x_init, prob_data.b_x_conti, 'v');
		prob_data.b_y_eq = stack(prob_data.b_y_init, prob_data.b_y_conti, 'v');
		prob_data.b_z_eq = stack(prob_data.b_z_init, prob_data.b_z_conti, 'v');
	}
	

	float thresold = prob_data.thresold;

	
	prob_data.rho_static_obs = 1.0;
	prob_data.rho_drone = 1.0;
	prob_data.rho_vel = 1.0;
	prob_data.rho_acc = 1.0;
	prob_data.rho_jerk = 1.0;
	prob_data.rho_snap = 1.0;
	prob_data.rho_ineq = 1.0;
	prob_data.rho_sfc = 1.0;
	
	// @ Lagrange Multiplier
	prob_data.lamda_x = Eigen :: ArrayXXd :: Ones(prob_data.nvar, 1) *0;
	prob_data.lamda_y = Eigen :: ArrayXXd :: Ones(prob_data.nvar, 1) *0;
	prob_data.lamda_z = Eigen :: ArrayXXd :: Ones(prob_data.nvar, 1) *0;

	// @ Position Constraints
	prob_data.s_x_ineq = Eigen :: ArrayXXd :: Ones(2*prob_data.num, 1) *0;
	prob_data.s_y_ineq = Eigen :: ArrayXXd :: Ones(2*prob_data.num, 1) *0;
	prob_data.s_z_ineq = Eigen :: ArrayXXd :: Ones(2*prob_data.num, 1) *0;

	
	if(prob_data.num_drone!=0)cost_drone = prob_data.A_drone.transpose().matrix() * prob_data.A_drone.matrix();
	if(prob_data.num_static_obs!=0)cost_static_obs = prob_data.A_static_obs.transpose().matrix() * prob_data.A_static_obs.matrix();
	
	if(prob_data.sfcc){
		if(prob_data.b_cp.rows() != 0)
			prob_data.s_cp = Eigen :: ArrayXXd :: Ones(prob_data.b_cp.rows(), 1) *0;
		else
			ROS_ERROR_STREAM("No convex polytope found");
	}

	int break_flag;
	for(int i = 0; i < prob_data.max_iter; i++){

		prob_data.prev_x = prob_data.x;
		prob_data.prev_y = prob_data.y;
		prob_data.prev_z = prob_data.z;

		break_flag = 0;

		// x - position, velocity, and accelertion constraints
		prob_data.b_x_vel = prob_data.d_vel * cos(prob_data.alpha_vel) * sin(prob_data.beta_vel);
		prob_data.b_x_acc = prob_data.d_acc * cos(prob_data.alpha_acc) * sin(prob_data.beta_acc);
		prob_data.B_x_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_x_ineq - prob_data.s_x_ineq).matrix();

		objective_x = prob_data.weight_goal * prob_data.cost_goal 
				+ prob_data.weight_smoothness * prob_data.cost_smoothness 
				+ prob_data.rho_vel * prob_data.cost_vel 
				+ prob_data.rho_acc * prob_data.cost_acc 
				+ prob_data.rho_ineq * prob_data.cost_ineq;

		lincost_x = -prob_data.lamda_x
					-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.x_ref.matrix()).array()
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_x_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_x_acc.matrix()).array()
					-prob_data.rho_ineq * prob_data.B_x_ineq;

		// x- jerk and snap constraints
		if(prob_data.jerk_snap_constraints){
			prob_data.b_x_jerk = prob_data.d_jerk * cos(prob_data.alpha_jerk) * sin(prob_data.beta_jerk);
			prob_data.b_x_snap = prob_data.d_snap * cos(prob_data.alpha_snap) * sin(prob_data.beta_snap);
			objective_x += prob_data.rho_jerk * prob_data.cost_jerk
				+ prob_data.rho_snap * prob_data.cost_snap;
			lincost_x += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_x_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_x_snap.matrix()).array();
		}

		// x - cylindrical static obstacle avoidance constraints
		if(prob_data.num_static_obs!=0){
			temp_x_static_obs =  prob_data.x_static_obs + prob_data.d_static_obs * cos(prob_data.alpha_static_obs) * prob_data.a_static_obs;
			prob_data.b_x_static_obs =  reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
			objective_x += prob_data.rho_static_obs * cost_static_obs;
			lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
		}

		// x - safe flight corridor constraints 
		if(!prob_data.sfcc){
			objective_x += prob_data.rho_sfc * prob_data.cost_sfc;
			temp_x_sfc =  (prob_data.x_anchor + prob_data.d_sfc * cos(prob_data.alpha_sfc)*sin(prob_data.beta_sfc));
			lincost_x += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * temp_x_sfc.matrix()).array();
		}
		else if(prob_data.sfcc ){
			temp_A_x_cp = prob_data.A_x_cp.matrix() * prob_data.P.matrix();
			objective_x += prob_data.rho_sfc * (temp_A_x_cp.transpose().matrix() * temp_A_x_cp.matrix()).array();
			prob_data.B_x_cp = temp_A_x_cp.transpose().matrix()*(-(prob_data.A_y_cp.matrix() * prob_data.y.matrix()).array() 
											- (prob_data.A_z_cp.matrix() * prob_data.z.matrix()).array() + prob_data.b_cp - prob_data.s_cp).matrix(); 
			lincost_x += -prob_data.rho_sfc * prob_data.B_x_cp;
		}

		// x - inter-agent collision avoidance constraints
		if(prob_data.num_drone!=0){
			temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.a_drone;
			prob_data.b_x_drone =  reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
			objective_x += prob_data.rho_drone * cost_drone;
			lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
		}

		// @ x - solve set of linear equations
		cost_x =  stack(stack(objective_x, prob_data.A_eq.transpose(), 'h'), 
							stack(prob_data.A_eq, Eigen::ArrayXXd::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
		cost_x_inv = (cost_x.matrix()).householderQr().solve(prob_data.I.matrix());
		sol_x = cost_x_inv.matrix() * stack(-lincost_x, prob_data.b_x_eq, 'v').matrix();
		primal_sol_x = sol_x.topRows(prob_data.nvar);
		prob_data.x = prob_data.P.matrix() * primal_sol_x.matrix();
		prob_data.xdot = prob_data.Pdot.matrix() * primal_sol_x.matrix();
		prob_data.xddot = prob_data.Pddot.matrix() * primal_sol_x.matrix();
		prob_data.xdddot = prob_data.Pdddot.matrix() * primal_sol_x.matrix();
		prob_data.xddddot = prob_data.Pddddot.matrix() * primal_sol_x.matrix();
		prob_data.x_up = prob_data.P_up.matrix() * primal_sol_x.matrix();
		prob_data.xdot_up = prob_data.Pdot_up.matrix() * primal_sol_x.matrix();
		prob_data.xddot_up = prob_data.Pddot_up.matrix() * primal_sol_x.matrix();


		// y - position, velocity, and accelertion constraints
		prob_data.b_y_vel = prob_data.d_vel * sin(prob_data.alpha_vel) * sin(prob_data.beta_vel);
		prob_data.b_y_acc = prob_data.d_acc * sin(prob_data.alpha_acc) * sin(prob_data.beta_acc);
		prob_data.B_y_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_y_ineq - prob_data.s_y_ineq).matrix();

		objective_y = prob_data.weight_goal * prob_data.cost_goal 
				+ prob_data.weight_smoothness * prob_data.cost_smoothness 
				+ prob_data.rho_vel * prob_data.cost_vel 
				+ prob_data.rho_acc * prob_data.cost_acc 
				+ prob_data.rho_ineq * prob_data.cost_ineq;
		
		lincost_y = -prob_data.lamda_y
					-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.y_ref.matrix()).array()
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_y_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_y_acc.matrix()).array()
					-prob_data.rho_ineq * prob_data.B_y_ineq;

		// y- jerk and snap constraints
		if(prob_data.jerk_snap_constraints){
			prob_data.b_y_jerk = prob_data.d_jerk * sin(prob_data.alpha_jerk) * sin(prob_data.beta_jerk);
			prob_data.b_y_snap = prob_data.d_snap * sin(prob_data.alpha_snap) * sin(prob_data.beta_snap);
			objective_y += prob_data.rho_jerk * prob_data.cost_jerk
				+ prob_data.rho_snap * prob_data.cost_snap;
			lincost_y += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_y_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_y_snap.matrix()).array();
		}

		// y - cylindrical static obstacle avoidance constraints
		if(prob_data.num_static_obs!=0){
			temp_y_static_obs =  prob_data.y_static_obs + prob_data.d_static_obs * sin(prob_data.alpha_static_obs) * prob_data.b_static_obs;
			prob_data.b_y_static_obs =  reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
			objective_y += prob_data.rho_static_obs * cost_static_obs;
			lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

		}

		// y - safe flight corridor constraints 
		if(!prob_data.sfcc){
			objective_y += prob_data.rho_sfc * prob_data.cost_sfc;
			temp_y_sfc =  (prob_data.y_anchor + prob_data.d_sfc * sin(prob_data.alpha_sfc)*sin(prob_data.beta_sfc));
			lincost_y += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * temp_y_sfc.matrix()).array();
		}
		else if(prob_data.sfcc ){
			temp_A_y_cp = prob_data.A_y_cp.matrix() * prob_data.P.matrix();
			objective_y += prob_data.rho_sfc * (temp_A_y_cp.transpose().matrix() * temp_A_y_cp.matrix()).array();
			prob_data.B_y_cp = temp_A_y_cp.transpose().matrix()*(-(prob_data.A_x_cp.matrix() * prob_data.x.matrix()).array() 
															-(prob_data.A_z_cp.matrix() * prob_data.z.matrix()).array() + prob_data.b_cp - prob_data.s_cp).matrix(); 
			lincost_y += -prob_data.rho_sfc * prob_data.B_y_cp; 
		}

		// y - inter-agent collision avoidance constraints
		if(prob_data.num_drone!=0){
			temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.b_drone;
			prob_data.b_y_drone =  reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
			objective_y += prob_data.rho_drone * cost_drone;
			lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();
		}

		// @ y - solve set of linear equations
		cost_y =  stack(stack(objective_y, prob_data.A_eq.transpose(), 'h'), 
							stack(prob_data.A_eq, Eigen::ArrayXXd::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
		cost_y_inv = (cost_y.matrix()).householderQr().solve(prob_data.I.matrix());
		sol_y = cost_y_inv.matrix() * stack(-lincost_y, prob_data.b_y_eq, 'v').matrix(); 
		primal_sol_y = sol_y.topRows(prob_data.nvar);
		prob_data.y = prob_data.P.matrix() * primal_sol_y.matrix();
		prob_data.ydot = prob_data.Pdot.matrix() * primal_sol_y.matrix();
		prob_data.yddot = prob_data.Pddot.matrix() * primal_sol_y.matrix();
		prob_data.ydddot = prob_data.Pdddot.matrix() * primal_sol_y.matrix();
		prob_data.yddddot = prob_data.Pddddot.matrix() * primal_sol_y.matrix();
		prob_data.y_up = prob_data.P_up.matrix() * primal_sol_y.matrix();
		prob_data.ydot_up = prob_data.Pdot_up.matrix() * primal_sol_y.matrix();
		prob_data.yddot_up = prob_data.Pddot_up.matrix() * primal_sol_y.matrix();

		// z - position, velocity, and accelertion constraints
		prob_data.b_z_vel = prob_data.d_vel * cos(prob_data.beta_vel);
		prob_data.b_z_acc = prob_data.d_acc * cos(prob_data.beta_acc);
		prob_data.B_z_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_z_ineq - prob_data.s_z_ineq).matrix();
		
		objective_z = prob_data.weight_goal * prob_data.cost_goal 
			+ prob_data.weight_smoothness * prob_data.cost_smoothness 
			+ prob_data.rho_vel * prob_data.cost_vel 
			+ prob_data.rho_acc * prob_data.cost_acc 
			+ prob_data.rho_ineq * prob_data.cost_ineq;
		
		lincost_z = -prob_data.lamda_z
					-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.z_ref.matrix()).array()
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_z_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_z_acc.matrix()).array()
					-prob_data.rho_ineq * prob_data.B_z_ineq;
		
		// z - jerk and snap constraints
		if(prob_data.jerk_snap_constraints){
			prob_data.b_z_jerk = prob_data.d_jerk * cos(prob_data.beta_jerk);
			prob_data.b_z_snap = prob_data.d_snap * cos(prob_data.beta_snap);
			objective_z += prob_data.rho_jerk * prob_data.cost_jerk
				+ prob_data.rho_snap * prob_data.cost_snap;
			lincost_z += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_z_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_z_snap.matrix()).array();
		}			

		// z - safe flight corridor constraints 
		if(!prob_data.sfcc){
			objective_z += prob_data.rho_sfc * prob_data.cost_sfc;
			temp_z_sfc =  (prob_data.z_anchor + prob_data.d_sfc * cos(prob_data.beta_sfc));			
			lincost_z += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * temp_z_sfc.matrix()).array();
		}
		else if(prob_data.sfcc ){
			temp_A_z_cp = prob_data.A_z_cp.matrix() * prob_data.P.matrix();
			objective_z += prob_data.rho_sfc * (temp_A_z_cp.transpose().matrix() * temp_A_z_cp.matrix()).array();
			prob_data.B_z_cp = temp_A_z_cp.transpose().matrix()*(-(prob_data.A_x_cp.matrix() * prob_data.x.matrix()).array() 
															-(prob_data.A_y_cp.matrix() * prob_data.y.matrix()).array() + prob_data.b_cp - prob_data.s_cp).matrix(); 
			lincost_z += -prob_data.rho_sfc * prob_data.B_z_cp; 
		}

		// z - inter-agent collision avoidance constraints
		if(prob_data.num_drone!=0){
			temp_z_drone =  prob_data.z_drone + prob_data.d_drone * cos(prob_data.beta_drone) * prob_data.c_drone;
			prob_data.b_z_drone =  reshape(temp_z_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
			objective_z += prob_data.rho_drone * cost_drone;
			lincost_z += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_z_drone.matrix()).array();
		}

		// @ z - solve a set of linear equations
		cost_z =  stack(stack(objective_z, prob_data.A_eq.transpose(), 'h'), 
					stack(prob_data.A_eq, Eigen::ArrayXXd::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
		cost_z_inv = (cost_z.matrix()).householderQr().solve(prob_data.I.matrix());
		sol_z = cost_z_inv.matrix() * stack(-lincost_z, prob_data.b_z_eq, 'v').matrix(); 
		primal_sol_z = sol_z.topRows(prob_data.nvar);
		prob_data.z = prob_data.P.matrix() * primal_sol_z.matrix();
		prob_data.zdot = prob_data.Pdot.matrix() * primal_sol_z.matrix();
		prob_data.zddot = prob_data.Pddot.matrix() * primal_sol_z.matrix();
		prob_data.zdddot = prob_data.Pdddot.matrix() * primal_sol_z.matrix();
		prob_data.zddddot = prob_data.Pddddot.matrix() * primal_sol_z.matrix();
		prob_data.z_up = prob_data.P_up.matrix() * primal_sol_z.matrix();
		prob_data.zdot_up = prob_data.Pdot_up.matrix() * primal_sol_z.matrix();
		prob_data.zddot_up = prob_data.Pddot_up.matrix() * primal_sol_z.matrix();
		
		// @ solve for polar and d variables
		initAlphaBeta(prob_data, VERBOSE);
		
		// @ update lagrange multipler and residuals
		// position, velocity, acceleration constraint residuals
		prob_data.s_x_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_x_ineq).max(0.0);
		res_x_ineq = (prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_x_ineq + prob_data.s_x_ineq;
		res_x_vel = prob_data.xdot - prob_data.d_vel * cos(prob_data.alpha_vel) * sin(prob_data.beta_vel);
		res_x_acc = prob_data.xddot - prob_data.d_acc * cos(prob_data.alpha_acc) * sin(prob_data.beta_acc);

		prob_data.lamda_x = prob_data.lamda_x 
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_x_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_x_acc.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_x_ineq.matrix()).array();

		prob_data.s_y_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_y_ineq).max(0.0);
		res_y_ineq = (prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_y_ineq + prob_data.s_y_ineq;
		res_y_vel = prob_data.ydot - prob_data.d_vel * sin(prob_data.alpha_vel) * sin(prob_data.beta_vel);
		res_y_acc = prob_data.yddot - prob_data.d_acc * sin(prob_data.alpha_acc) * sin(prob_data.beta_acc);

		prob_data.lamda_y = prob_data.lamda_y 
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_y_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_y_acc.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_y_ineq.matrix()).array();

		prob_data.s_z_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_z.matrix()).array() + prob_data.b_z_ineq).max(0.0);
		res_z_ineq = (prob_data.A_ineq.matrix() * primal_sol_z.matrix()).array() - prob_data.b_z_ineq + prob_data.s_z_ineq;
		res_z_vel = prob_data.zdot - prob_data.d_vel * cos(prob_data.beta_vel);
		if(!prob_data.use_thrust_values)
			res_z_acc = prob_data.zddot - prob_data.d_acc * cos(prob_data.beta_acc);
		else
			res_z_acc = prob_data.zddot + prob_data.gravity - prob_data.d_acc * cos(prob_data.beta_acc);
		
		prob_data.lamda_z = prob_data.lamda_z 
				-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_z_vel.matrix()).array()
				-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_z_acc.matrix()).array()
				-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_z_ineq.matrix()).array();
	
		prob_data.res_x_ineq_norm = res_x_ineq.matrix().norm();
		prob_data.res_x_vel_norm = res_x_vel.matrix().norm();
		prob_data.res_x_acc_norm = res_x_acc.matrix().norm();

		prob_data.res_y_ineq_norm = res_y_ineq.matrix().norm();
		prob_data.res_y_vel_norm = res_y_vel.matrix().norm();
		prob_data.res_y_acc_norm = res_y_acc.matrix().norm();

		prob_data.res_z_ineq_norm = res_z_ineq.matrix().norm();
		prob_data.res_z_vel_norm = res_z_vel.matrix().norm();
		prob_data.res_z_acc_norm = res_z_acc.matrix().norm();
		
		if((prob_data.res_x_ineq_norm > thresold || prob_data.res_y_ineq_norm > thresold || prob_data.res_z_ineq_norm > thresold) && prob_data.rho_ineq){;
				prob_data.rho_ineq *= prob_data.delta_ineq;
				if(prob_data.rho_ineq > prob_data.rho_ineq_max) prob_data.rho_ineq = prob_data.rho_ineq_max;
		}
		else{
			prob_data.rho_ineq /= prob_data.delta_ineq;
			if(prob_data.rho_ineq < 1) prob_data.rho_ineq = 1.0;
			break_flag++;
		}
		
		if(prob_data.res_x_vel_norm > thresold || prob_data.res_y_vel_norm > thresold || prob_data.res_z_vel_norm > thresold){;
				prob_data.rho_vel *= prob_data.delta_vel;
				if(prob_data.rho_vel > prob_data.rho_vel_max) prob_data.rho_vel = prob_data.rho_vel_max;
		}
		else{
			prob_data.rho_vel /= prob_data.delta_vel;
			if(prob_data.rho_vel < 1) prob_data.rho_vel = 1.0;
			break_flag++;
		}
		if(prob_data.res_x_acc_norm > thresold || prob_data.res_y_acc_norm > thresold || prob_data.res_z_acc_norm > thresold){;
				prob_data.rho_acc *= prob_data.delta_acc;
				if(prob_data.rho_acc > prob_data.rho_acc_max) prob_data.rho_acc = prob_data.rho_acc_max;
		}
		else{
			prob_data.rho_acc /= prob_data.delta_acc;
			if(prob_data.rho_acc < 1) prob_data.rho_acc = 1.0;
			break_flag++;
		}

		// jerk and snap constraints residuals
		if(prob_data.jerk_snap_constraints){
			res_x_jerk = prob_data.xdddot - prob_data.d_jerk * cos(prob_data.alpha_jerk) * sin(prob_data.beta_jerk);
			res_x_snap = prob_data.xddddot - prob_data.d_snap * cos(prob_data.alpha_snap) * sin(prob_data.beta_snap);
			prob_data.lamda_x += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_x_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_x_snap.matrix()).array();

			res_y_jerk = prob_data.ydddot - prob_data.d_jerk * sin(prob_data.alpha_jerk) * sin(prob_data.beta_jerk);
			res_y_snap = prob_data.yddddot - prob_data.d_snap * sin(prob_data.alpha_snap) * sin(prob_data.beta_snap);
			prob_data.lamda_y += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_y_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_y_snap.matrix()).array();

			res_z_jerk = prob_data.zdddot - prob_data.d_jerk * cos(prob_data.beta_jerk);
			res_z_snap = prob_data.zddddot - prob_data.d_snap * cos(prob_data.beta_snap);
			prob_data.lamda_z += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_z_jerk.matrix()).array()
				-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_z_snap.matrix()).array();

			prob_data.res_x_jerk_norm = res_x_jerk.matrix().norm();
			prob_data.res_y_jerk_norm = res_y_jerk.matrix().norm();
			prob_data.res_z_jerk_norm = res_z_jerk.matrix().norm();

			prob_data.res_x_snap_norm = res_x_snap.matrix().norm();
			prob_data.res_y_snap_norm = res_y_snap.matrix().norm();
			prob_data.res_z_snap_norm = res_z_snap.matrix().norm();

			if(prob_data.res_x_jerk_norm > thresold || prob_data.res_y_jerk_norm > thresold || prob_data.res_z_jerk_norm > thresold){;
				prob_data.rho_jerk *= prob_data.delta_jerk;
				if(prob_data.rho_jerk > prob_data.rho_jerk_max) prob_data.rho_jerk = prob_data.rho_jerk_max;
			}
			else {
				prob_data.rho_jerk /= prob_data.delta_jerk;
				if(prob_data.rho_jerk < 1) prob_data.rho_jerk = 1.0;
				break_flag++;
			}

			if(prob_data.res_x_snap_norm > thresold || prob_data.res_y_snap_norm > thresold || prob_data.res_z_snap_norm > thresold){;
					prob_data.rho_snap *= prob_data.delta_snap;
					if(prob_data.rho_snap > prob_data.rho_snap_max) prob_data.rho_snap = prob_data.rho_snap_max;
			}
			else {
				prob_data.rho_snap /= prob_data.delta_snap;
				if(prob_data.rho_snap < 1) prob_data.rho_snap = 1.0;
				break_flag++;
			}
		}
		else{
			prob_data.res_x_jerk_norm = 0.0;
			prob_data.res_y_jerk_norm = 0.0;
			prob_data.res_z_jerk_norm = 0.0;

			prob_data.res_x_snap_norm = 0.0;
			prob_data.res_y_snap_norm = 0.0;
			prob_data.res_z_snap_norm = 0.0;

			break_flag += 2;
		}

		// cylindircal static obstacle avoidance constraints
		if(prob_data.num_static_obs!=0){
			
			res_x_static_obs = reshape(((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_static_obs * prob_data.d_static_obs * cos(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
			prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_x_static_obs.matrix()).array();

			res_y_static_obs = reshape(((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_static_obs * prob_data.d_static_obs * sin(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
			prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_y_static_obs.matrix()).array();
		
			prob_data.res_x_static_obs_norm = res_x_static_obs.matrix().norm();
			prob_data.res_y_static_obs_norm = res_y_static_obs.matrix().norm();
		
			if(prob_data.res_x_static_obs_norm > thresold || prob_data.res_y_static_obs_norm > thresold){
				prob_data.rho_static_obs *= prob_data.delta_static_obs;
				if(prob_data.rho_static_obs > prob_data.rho_static_obs_max) prob_data.rho_static_obs = prob_data.rho_static_obs_max;
			}
			else {
				prob_data.rho_static_obs /= prob_data.delta_static_obs;
				if(prob_data.rho_static_obs < 1) prob_data.rho_static_obs = 1.0;
				break_flag++;
			}
		}
		else{
			prob_data.res_x_static_obs_norm = 0;
			prob_data.res_y_static_obs_norm = 0;
			break_flag++;
		}

		// safe flight corridor constraint residulas
		if(!prob_data.sfcc){
			res_x_sfc = prob_data.x - prob_data.x_anchor - prob_data.d_sfc * cos(prob_data.alpha_sfc)*sin(prob_data.beta_sfc);
			prob_data.lamda_x += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * res_x_sfc.matrix()).array();

			res_y_sfc = prob_data.y - prob_data.y_anchor - prob_data.d_sfc * sin(prob_data.alpha_sfc)*sin(prob_data.beta_sfc);
			prob_data.lamda_y += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * res_y_sfc.matrix()).array();

			res_z_sfc = prob_data.z - prob_data.z_anchor - prob_data.d_sfc * cos(prob_data.beta_sfc);
			prob_data.lamda_z += -prob_data.rho_sfc * (prob_data.P.transpose().matrix() * res_z_sfc.matrix()).array();

			prob_data.res_x_sfc_norm = res_x_sfc.matrix().norm();
			prob_data.res_y_sfc_norm = res_y_sfc.matrix().norm();
			prob_data.res_z_sfc_norm = res_z_sfc.matrix().norm();

			if(prob_data.res_x_sfc_norm > thresold || prob_data.res_y_sfc_norm > thresold || prob_data.res_z_sfc_norm > thresold){
				prob_data.rho_sfc *= prob_data.delta_sfc;
				if(prob_data.rho_sfc > prob_data.rho_sfc_max) prob_data.rho_sfc = prob_data.rho_sfc_max;
			}
			else {
				prob_data.rho_sfc /= prob_data.delta_sfc;
				if(prob_data.rho_sfc < 1) prob_data.rho_sfc = 1.0;
				break_flag++;
			}
		}
		else if(prob_data.sfcc ){
			
			prob_data.s_cp = ((-(prob_data.A_x_cp.matrix() * prob_data.x.matrix() + prob_data.A_y_cp.matrix() * prob_data.y.matrix() + prob_data.A_z_cp.matrix() * prob_data.z.matrix())).array() + prob_data.b_cp).max(0.0);

			res_x_sfc = (prob_data.A_x_cp.matrix() * prob_data.x.matrix() + prob_data.A_y_cp.matrix() * prob_data.y.matrix() + prob_data.A_z_cp.matrix() * prob_data.z.matrix()).array() - prob_data.b_cp + prob_data.s_cp;
			prob_data.lamda_x += -prob_data.rho_sfc * (temp_A_x_cp.transpose().matrix() * res_x_sfc.matrix()).array();
			prob_data.res_x_sfc_norm = res_x_sfc.matrix().norm();

			res_y_sfc = res_x_sfc;//(prob_data.A_x_cp.matrix() * prob_data.x.matrix() + prob_data.A_y_cp.matrix() * prob_data.y.matrix() + prob_data.A_z_cp.matrix() * prob_data.z.matrix()).array() - prob_data.b_cp + prob_data.s_cp;
			prob_data.lamda_y += -prob_data.rho_sfc * (temp_A_y_cp.transpose().matrix() * res_y_sfc.matrix()).array();
			prob_data.res_y_sfc_norm = res_y_sfc.matrix().norm();

			res_z_sfc = res_x_sfc;//(prob_data.A_x_cp.matrix() * prob_data.x.matrix() + prob_data.A_y_cp.matrix() * prob_data.y.matrix() + prob_data.A_z_cp.matrix() * prob_data.z.matrix()).array() - prob_data.b_cp + prob_data.s_cp;
			prob_data.lamda_z += -prob_data.rho_sfc * (temp_A_z_cp.transpose().matrix() * res_z_sfc.matrix()).array();
			prob_data.res_z_sfc_norm = res_z_sfc.matrix().norm();
			
			if(prob_data.res_x_sfc_norm > thresold || prob_data.res_y_sfc_norm > thresold || prob_data.res_z_sfc_norm > thresold){
				prob_data.rho_sfc *= prob_data.delta_sfc;
				if(prob_data.rho_sfc > prob_data.rho_sfc_max) prob_data.rho_sfc = prob_data.rho_sfc_max;
			}
			else{
				prob_data.rho_sfc /= prob_data.delta_sfc;
				if(prob_data.rho_sfc < 1.0) prob_data.rho_sfc = 1.0;
				break_flag++;
			}
		}

		// inter-agent collision avoidance constraint residuals
		if(prob_data.num_drone!=0){
			
			res_x_drone = reshape(((-prob_data.x_drone).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_drone * prob_data.d_drone * cos(prob_data.alpha_drone) * sin(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
			prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_x_drone.matrix()).array();

			res_y_drone = reshape(((-prob_data.y_drone).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_drone * prob_data.d_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
			prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_y_drone.matrix()).array();

			res_z_drone = reshape(((-prob_data.z_drone).rowwise() + prob_data.z.transpose().row(0) - prob_data.c_drone * prob_data.d_drone * cos(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
			prob_data.lamda_z = prob_data.lamda_z -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_z_drone.matrix()).array();

			prob_data.res_x_drone_norm = res_x_drone.matrix().norm();
			prob_data.res_y_drone_norm = res_y_drone.matrix().norm();
			prob_data.res_z_drone_norm = res_z_drone.matrix().norm();

			if(prob_data.res_x_drone_norm > thresold || prob_data.res_y_drone_norm > thresold || prob_data.res_z_drone_norm > thresold){
				prob_data.rho_drone *= prob_data.delta_drone;
				if(prob_data.rho_drone > prob_data.rho_drone_max) prob_data.rho_drone = prob_data.rho_drone_max;
			}
			else {
				prob_data.rho_drone /= prob_data.delta_drone;
				if(prob_data.rho_drone < 1) prob_data.rho_drone = 1.0;
				break_flag++;
			}
		}
		else{
			prob_data.res_x_drone_norm = 0;
			prob_data.res_y_drone_norm = 0;
			prob_data.res_z_drone_norm = 0;
			break_flag++;
		}
		
		if(break_flag == 8)
			break;	
	}
	
	if(break_flag != 8){
			prob_data.weight_goal *= prob_data.delta_aggressive;
			//prob_data.weight_smoothness *= prob_data.delta_aggressive;
		}
	else{
		prob_data.weight_goal = prob_data.weight_goal_og;
		prob_data.weight_smoothness = prob_data.weight_smoothness_og;

		if(prob_data.weight_goal > prob_data.weight_goal_og)
			prob_data.weight_goal = prob_data.weight_goal_og;

		if(prob_data.weight_goal > prob_data.weight_smoothness_og)
			prob_data.weight_smoothness = prob_data.weight_smoothness_og;
	}					
}
