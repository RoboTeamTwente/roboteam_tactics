#pragma once

#include "roboteam_tactics/Parts.h"
#include "GoToPos.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class KeepPosition : public Skill {
public:
	KeepPosition(std::string name, bt::Blackboard::Ptr bb);
	Status Update() override;
    const float MINIMUM_ROBOT_DISTANCE = 0.5;
private:
	std::unique_ptr<GoToPos> gtp;
	bool updateGoalPosition();
	Vector2 getNearestObject(Vector2 ownPos) const;
};

}
