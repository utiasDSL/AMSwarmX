#pragma once
#include "amswarmx/trajectory_utils.hpp"
#include "amswarmx/solve_polar_var.hpp"

// Solve position3D
void computeXYZ(probData &prob_data, int VERBOSE);

// Solve position2D
void computeXY(probData &prob_data, int VERBOSE);