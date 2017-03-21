#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class CanSeeRobot
 * \brief See YAML
 */
/*
 * Descr: Checks whether one robot can see another
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: One of the robots
 *   - targetID:
 *       Type: Int
 *       Descr: The other robot
 */
class CanSeeRobot : public Condition {
    public:
    CanSeeRobot(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();

} ;

}
