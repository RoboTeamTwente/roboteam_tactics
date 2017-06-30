#pragma once

#include "roboteam_tactics/Parts.h"
#include "RotateAroundPoint.h"

namespace rtt {

class KickPenalty : public Skill {
public:
	static constexpr double AIM_SAFETY_MARGIN = .03; // aim 3 cm from corner
	static constexpr double KICK_SAFETY_MARGIN = .5; // Kick velocity = max (8.0) - this constant
	static constexpr int KICK_COUNT = 10; // Amount of kick commands to send before succeeding.

	KickPenalty(std::string name, bt::Blackboard::Ptr bb);
	Status Update() override;
private:
	bool choseTarget;
	bool shootingAtTopCorner;
	bool rotateDone;
	int kickCount;
	std::unique_ptr<RotateAroundPoint> rotator;
};

}


