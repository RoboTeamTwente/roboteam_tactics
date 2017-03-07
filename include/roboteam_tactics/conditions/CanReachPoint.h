#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
    
class CanReachPoint : public Condition {

public:
    CanReachPoint(std::string name, bt::Blackboard::Ptr blackboard);  
    // Vector2 ComputeMaxAcceleration(double angle, Vector2 maxAcc);
    double estimateTimeToPoint(Vector2 currentPos, Vector2 currentVel, Vector2 targetPos);
    Status Update();
    
    std::string node_name() { return "CanReachPoint"; }
private:

	double maxAcc = 2;
	double maxDec = 4;
	double maxVel = 2;
	// double maxAngAcc = 10;
	// double maxAngVel = 4;
	double posPGain = 2.0;
	double decelerationDistance = 0.2; // 
    
};

} // rtt
