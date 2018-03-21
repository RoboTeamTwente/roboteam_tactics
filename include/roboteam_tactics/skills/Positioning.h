#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/utils/OpportunityFinder.h"

namespace rtt {

/**
 * \class Positioning
 * \brief See YAML
 */
/*
 * Descr: Position the robot in the best position according to a certain profile
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 */
class Positioning : public Skill {
public:
	Positioning(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

	void Initialize() override;

	Status Update() override;

	static VerificationMap required_params() {
		VerificationMap params;
        	params["ROBOT_ID"] = BBArgumentType::Int;
        	return params;
	}
	std::string node_name() { return "Positioning"; }
private:
	uint ROBOT_ID;
	time_point start;
	Vector2 bestPosition;
	int counter;
	bool dontGoToPos;

	OpportunityFinder opportunityFinder;
	GoToPos goToPos;
};


} // rtt
