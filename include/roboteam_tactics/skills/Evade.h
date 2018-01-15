#pragma once

#include "roboteam_tactics/Parts.h"
#include "GoToPos.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

namespace rtt {

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
};

}
