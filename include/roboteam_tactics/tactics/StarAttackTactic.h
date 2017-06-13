#pragma once

#include "roboteam_tactics/Parts.h"
#include <vector>
#include <map>
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Section.h"
#include "boost/optional.hpp"

namespace rtt {

class StarAttackTactic : public Tactic {
public:
	StarAttackTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize() override;
	Status Update() override;
	void Terminate(Status) override;

private:
	static constexpr double DEVIATION_RANGE = .25;

	struct BotParams {
		Vector2 tgtPos;
		boost::uuids::uuid token;
		roboteam_msgs::RoleDirective rd;
	};

	std::vector<Vector2> basePositions;
	std::map<int, BotParams> botParams;
	int freeKickTaker;
    time_point start;
    bool canRun;


	void initBot(int botID, int posID);
};

static constexpr std::array<Vector2, 3> STAR_POSITION_FRACTIONS = {
				      	  	 // On full-sized field:
	Vector2{ .733,  .175 },  // { 3.3,  .7 }
	Vector2{ .711, -.175 },  // { 3.2, -.7 }
	Vector2{ .822, -.333 }   // { 3.7, -1  }
};

}
