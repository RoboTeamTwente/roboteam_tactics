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
 *
 *   - waitAtDistance:
 *       Type: Double
 *       Descr: The robot will wait at a set distance from the position it claimed, to prevent anticipation by the opponent
 *
 *   - dontUnclaim:
 *       Type: Bool
 *       Descr: If true, when terminate gets called for this skill, it wont unclaim its claimed position.
 */
class Positioning : public Skill {
public:
	Positioning(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

	void Initialize() override;

	Status Update() override;
	void Terminate(Status s) override;

	static VerificationMap required_params() {
		VerificationMap params;
        	params["ROBOT_ID"] = BBArgumentType::Int;
        	return params;
	}
	std::string node_name() { return "Positioning"; }
private:
	uint robotID;
	time_point start;
	Vector2 bestPosition;
	int counter;
	double initialBoxSize = 9.0;
	Vector2 initialPos;

	OpportunityFinder opportunityFinder;
	GoToPos goToPos;

	double getBallGoalHalfwayAngle(Vector2 testPos);
};


} // rtt
