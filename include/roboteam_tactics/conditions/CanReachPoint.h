#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_utils/Vector2.h"
#include <boost/optional.hpp>

namespace rtt {
    
class CanReachPoint : public Condition {

public:
    CanReachPoint(std::string name, bt::Blackboard::Ptr blackboard);  
    roboteam_utils::Vector2 ComputeMaxAcceleration(double angle, roboteam_utils::Vector2 maxAcc);
    double cleanAngle(double angle);
    Status Update();
private:
};
    
}