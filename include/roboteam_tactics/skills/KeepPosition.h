#pragma once

#include "roboteam_tactics/Parts.h"
#include "GoToPos.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

namespace rtt {

/**
 * \class KeepPosition
 * \brief See YAML
 */
/**
 * # Using the YAML multiline literal here
 * Descr: TODO
 * Params:
 *   - ROBOT_ID:
 *       Descr:     Id of the robot
 *       Type:      Int
 */
 
class KeepPosition : public Skill {
public:
	KeepPosition(std::string name, bt::Blackboard::Ptr bb);
	Status Update() override;
    constexpr static double MINIMUM_ROBOT_DISTANCE = 0.5;
private:
	std::unique_ptr<GoToPos> gtp;
	bool updateGoalPosition();
	Vector2 getNearestObject(Vector2 ownPos);
	boost::optional<Position> initialPos;
};

}
