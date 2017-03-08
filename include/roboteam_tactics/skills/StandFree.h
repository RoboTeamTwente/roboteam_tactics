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
 *       Descr: The id of the robot
 *   - theirID:
 *       Type: Int
 *       Used when: xGoal or yGoal is not set
 *       Descr: The id of the robot who should be able to pass to the current one
 *   - whichTeam:
 *       Type: String
 *       Can be:
 *         - us: Our team
 *         - them: Their team
 *       Used when: xGoal or yGoal is not set
 *       Descr: What team the robot with id theirID is on
 *   - xGoal:
 *       Type: Double
 *       Descr: The x-coordinate of the point to maintain line of sight to
 *   - yGoal:
 *       Type: Double
 *       Descr: The y-coordinate of the point to maintain line of sight to
 *   - distanceFromPoint:
 *       Type: Double
 *       Descr: The distance to maintain to other robots
 */
class StandFree : public Skill {

public:
    StandFree(std::string name, bt::Blackboard::Ptr blackboard);  
	boost::optional<Cone> MakeCoverCone(std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots, Vector2 myPos, Vector2 targetPos);
    Status Update();
private:
	GoToPos goToPos;
    Draw drawer;
};
    
}