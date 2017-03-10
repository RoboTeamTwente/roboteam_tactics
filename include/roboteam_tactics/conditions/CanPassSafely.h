#pragma once

#include <boost/optional.hpp>

#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class CanPassSafely
 * \brief See YAML
 */    
/*
 * Descr: Checks whether one robot can pass to another with little chance of the ball being intercepted.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The passing robot
 *   - passToRobot:
 *       Type: Int
 *       Descr: The receiving robot
 */
class CanPassSafely : public Condition {

public:
    CanPassSafely(std::string name, bt::Blackboard::Ptr blackboard);  
    Status Update();
    
    std::string node_name() { return "CanPassSafely"; }
private:
};
    
} // rtt
