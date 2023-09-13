#pragma once
#include "amswarmx/obstacles_utils.hpp" 
#include "amswarmx/run_grid_planner.hpp"
#include "amswarmx/select_anchor_goal.hpp"
#include "amswarmx/solve_polar_var.hpp"
#include "amswarmx/solve_position_var.hpp"
#include "amswarmx/trajectory_utils.hpp"

// Run optimizer
void checkResiduals(probData &prob_data, int VERBOSE);
void initializeOptimizer(probData &prob_data, int VERBOSE);
void deployAgent(probData &prob_data, int VERBOSE);