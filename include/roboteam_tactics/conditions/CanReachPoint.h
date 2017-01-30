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
    // roboteam_utils::Vector2 ComputeMaxAcceleration(double angle, roboteam_utils::Vector2 maxAcc);
    double estimateTimeToPoint(roboteam_utils::Vector2 currentPos, roboteam_utils::Vector2 currentVel, roboteam_utils::Vector2 targetPos);
    Status Update();
    
    std::string node_name() { return "CanReachPoint"; }
private:

	double maxAcc = 2;
	double maxDec = 4;
	double maxVel = 2;
	// double maxAngAcc = 10;
	// double maxAngVel = 4;
	double posPGain = 3.0;
	double decelerationDistance = 0.2; // 
    
};

} // rtt