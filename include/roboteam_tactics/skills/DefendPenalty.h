#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class DefendPenalty
 * \brief See YAML
 */
/**
 * Descr: Defender specifically for penalties
 * 
 * Params:
 *  - ROBOT_ID:
 *     Descr:     Id of the robot
 *     Type:      Int
 *
 *  - isShortRangePenalty:
 *     Descr:     Indicates whether the penalty range is short or long
 *     Type:      Bool
 */
 
class DefendPenalty : public Skill {
public:
	static constexpr double SHORT_RANGE_PENALTY_DIST = 1.0;
	static constexpr double LONG_RANGE_PENALTY_DIST = 6.0;

	// When we are 30 cm from the center line, we can still reach the corner of the goal.
	static constexpr double MAX_DIST_FROM_CENTER_FOR_CLOSE_PENALTY = 0.3;
	static constexpr double MAX_DIST_FROM_CENTER_FOR_FAR_PENALTY = 0.1;

	DefendPenalty(std::string name, bt::Blackboard::Ptr bb);
	Status Update() override;

private:
	std::unique_ptr<GoToPos> gtp;
	bool ballUnderway;
};

}

