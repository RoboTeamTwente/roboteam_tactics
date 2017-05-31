#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/AimAt.h"

namespace rtt {

class SelectStarAttackShooter : public Skill {
public:
	SelectStarAttackShooter(std::string name = "", bt::Blackboard::Ptr bb = nullptr);
	Status Update() override;
	void Initialize() override;
private:

	// How wide of an opening a teammate needs to have before I pick one.
	static constexpr double MIN_DESIRED_GOAL_OPENING = .1;

	static constexpr std::chrono::nanoseconds MAX_WAIT {3000000000ull};

	std::set<int> available;
	time_point start;
	std::shared_ptr<AimAt> aimAt;

	boost::optional<std::pair<int, Section>> selectBest() const;
	std::map<int, boost::optional<Section>> largestOpenSections() const;
	bool hasBall();
};

}
