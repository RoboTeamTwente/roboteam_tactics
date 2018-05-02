#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class IsBallInZone
 * \brief See YAML
 */
/*
 * Descr: Checks whether the ball or robot is in a defence zone
 * Params:
 *   - zone:
 *       Type: Int
 *       Can Be:
 *         - 1: As seen from the goal: left rear
 *         - 2: As seen from the goal: right rear
 *         - 3: All field except area close to opponents goal
 *         - 4: Everywhere except the edges of the field (where the robot cant get the ball in case of demo field)
 *         - 5: Zone 4 with larger margin
 *       Descr: What zone to check
 *
 *    - x1:
 *       Type: Double
 *       Descr: X coordinate of the top left point of the zone
 *
 *    - y1:
 *       Type: Double
 *       Descr: Y coordinate of the top left point of the zone
 *
 *    - x2:
 *       Type: Double
 *       Descr: X coordinate of the bottom right point of the zone
 *
 *    - y2:
 *       Type: Double
 *       Descr: Y coordinate of the bottom right point of the zone
 *
 *   - robot:
 *       Type: Bool
 *       Descr: Whether this condition applies to the ball or the robot
 */
class IsInZone : public Condition {
public:
    IsInZone(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
private:
	ros::NodeHandle n;
} ;

}
