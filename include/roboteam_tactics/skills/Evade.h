#pragma once

#include "roboteam_tactics/Parts.h"
#include "GoToPos.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Draw.h"

namespace rtt {

/**
 * \class Evade
 * \brief See YAML
 */
/**
 * Descr: Aims the robot at a certain point.
 * 
 * Params:
 * - ROBOT_ID:
 *     Descr:     Id of the robot
 *     Type:      Int
 * 
 * - KEEPER_ID:
 *     Descr:     Id of the keeper
 *     Type:      Int
 *
 * - returnToInitialPos:
 *     Type: Bool
 *     Descr: Whether to return to the original position
 *     Note: Is not used at the moment
 */
 
class Evade : public Skill {
public:
	Evade(std::string name, bt::Blackboard::Ptr bb);
	Status Update() override;
    constexpr static double LOOKING_DISTANCE = 1;
private:
	std::unique_ptr<GoToPos> gtp;
	bool updateGoalPosition();
	Vector2 getNearestObject(Vector2 ownPos);
	boost::optional<Position> initialPos;
	Draw drawer;
};

}
