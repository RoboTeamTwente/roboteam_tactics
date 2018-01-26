#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/AimAt.h"

namespace rtt {

/**
 * \class SelectStarAttackShooter
 * \brief See YAML
 */
/**
 * # Using the YAML multiline literal here
 * Descr: Goes to a position
 * Params:
 *   - ROBOT_ID:
 *       Descr:     Id of the robot
 *       Type:      Int
 *
 *   - availableIDs:
 *       Descr:     The IDs of the available robots split by ','
 *       Type:      String
 *
 *   - idStoreParam:
 *       Descr:     The ID of the param in which you want to store the starattackshooter
 *       Type:      String
 */
 
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
