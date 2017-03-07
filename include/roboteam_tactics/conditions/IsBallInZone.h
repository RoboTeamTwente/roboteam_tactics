#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class IsBallInZone
 * \brief See YAML
 */
/*
 * Descr: Checks whether the ball is in a defence zone
 * Params:
 *   zone:
 *     Type: Int
 *     Can Be:
 *       one: our side
 *       two: their side
 *     Descr: What zone to check
 */
class IsBallInZone : public Condition {
public:
    IsBallInZone(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
private:
	ros::NodeHandle n;
} ;

}
