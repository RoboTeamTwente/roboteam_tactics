#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point);

/**
 * \class IsBallInDefenseArea
 * \brief See YAML
 */
/*
 * Descr: Checks the ball is in one of the goals
 * Params:
 *   - our_goal:
 *       Type: Bool
 *       Descr: Whether to check our defense area, or the opponents'
 */
class IsBallInDefenseArea : public Condition {
public:
    IsBallInDefenseArea(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    
    Status Update();
private:
} ;

} // rtt