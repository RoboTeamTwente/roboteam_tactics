#pragma once

#include <boost/optional.hpp>

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Cone.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/Draw.h"

namespace rtt {

/**
 * \class StandFree
 * \brief See YAML
 */
/*
 * Descr: Maintains line of sight between a robot and either another robot or a set point
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The ID of the robot
 *
 *   - KEEPER_ID:
 *       Type: Int
 *       Descr: The ID of the keeper
 *
 *   - theirID:
 *       Type: Int
 *       Descr: The ID of the robot we should remain in line of sight of
 *
 *   - ourTeam:
 *       Type: Bool
 *       Descr: True if the robot specified with theirID is on our team, defaults to true
 *
 *   - xGoal:
 *       Type: Double
 *       Descr: The x-coordinate of the point we should find a free spot close to (and drive towards if it is free)
 *
 *   - yGoal:
 *       Type: Double
 *       Descr: The y-coordinate of the point we should find a free spot close to (and drive towards if it is free)
 *
 *   - closeToX:
 *       Type: Double
 *       Descr: The x-coordinate of the point we should try to find a free position as close as possible to (for example the position of their goal)
 *
 *   - closeToY:
 *       Type: Double
 *       Descr: The y-coordinate of the point we should try to find a free position as close as possible to (for example the position of their goal)
 *
 *   - distanceFromPoint:
 *       Type: Double
 *       Descr: The perpendicular distance between the line between the two robots, and robots from the opposing team
 */

class StandFree : public Skill {

public:
    StandFree(std::string name, bt::Blackboard::Ptr blackboard);  
	boost::optional<Cone> MakeCoverCone(std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots, Vector2 myPos, Vector2 targetPos);
    Status Update();
private:
	GoToPos goToPos;
    Draw drawer;
    Vector2 targetPos;
};
    
}
