#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class CanSeeTheirGoal : public Condition {
 public:
    CanSeeTheirGoal(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() ;
private:
	ros::NodeHandle n;
} ;

}
