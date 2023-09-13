#pragma once
#include "amswarmx/trajectory_utils.hpp"

// Obstacles
void initObstacles(probData &prob_data, int VERBOSE);
void neigbhoringAgents(probData &prob_data, int VERBOSE);
void getConvexPolytope2D(probData &prob_data, int VERBOSE);
void getConvexPolytope3D(probData &prob_data, int VERBOSE);
void castRobot(probData &prob_data, octomap::point3d &origin, octomap::point3d &direction, octomap::point3d &ray_end, double &d_sfc);
Eigen :: ArrayXXd queryDistances(probData &prob_data, int VERBOSE);