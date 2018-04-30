#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point);
bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point, double safetyMargin);

/**
 * \class IsInDefenseArea
 * \brief See YAML
 */
/*
 * Descr: Checks if the ball or robot is in one of the goals
 * Params:
 *   - ourDefenseArea:
 *       Type: Bool
 *       Descr: Whether to check our defense area, or the opponents'
 *   - robot:
 *       Type: Bool
 *       Descr: Whether this condition applies to the ball or the robot
 *
 *   - margin:
 *       Type: Double
 */
class IsInDefenseArea : public Condition {
public:
    IsInDefenseArea(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

    Status Update();
private:
} ;

} // rtt
