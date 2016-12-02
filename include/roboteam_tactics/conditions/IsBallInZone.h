#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class IsBallInZone : public Condition {
public:
    IsBallInZone(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
private:
	ros::NodeHandle n;
} ;

}
