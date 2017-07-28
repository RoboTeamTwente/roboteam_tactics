#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class IsBallMovingTowardsGoal
 * \brief See YAML
 */
/*
 * Descr: Checks the ball is in one of the goals
 * Params:
 *   - our_goal:
 *       Type: Bool
 *       Descr: Whether to check our goal, or the opponents'
 */
class IsBallMovingTowardsGoal : public Condition {
public:
    IsBallMovingTowardsGoal(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
private:
	ros::NodeHandle n;
} ;

}
