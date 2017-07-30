#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

class CanKeeperMove : public Condition {

public:
	CanKeeperMove(std::string, bt::Blackboard::Ptr);
	Status Update() override;
private:
	boost::optional<Vector2> initialBallPos;
};

}
