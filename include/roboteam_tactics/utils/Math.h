
#pragma once
#include "ros/ros.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/Math.h"
#include <cmath>

namespace rtt {
    
double cleanAngle(double angle);
roboteam_utils::Vector2 worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation);
double computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos);
    
}
