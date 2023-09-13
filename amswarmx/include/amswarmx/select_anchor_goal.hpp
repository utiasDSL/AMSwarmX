#pragma once
#include "amswarmx/trajectory_utils.hpp"


// Anchor and Goal selection
void setGoal(probData &prob_data, int VERBOSE);
void assignAnchorPoints(probData &prob_data, int VERBOSE);