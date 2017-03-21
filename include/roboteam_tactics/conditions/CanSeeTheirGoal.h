#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class CanSeeTheirGoal
 * \brief See YAML
 */
/*
 * Descr: Checks whether a robot can see the opponents' goal.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The robot to check for
 */
class CanSeeTheirGoal : public Condition {
 public:
    CanSeeTheirGoal(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() ;
private:
	ros::NodeHandle n;
} ;

}
