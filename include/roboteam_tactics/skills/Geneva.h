#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

/**
 * \class Geneva
 * \brief See YAML
 */
/*
 * Descr: Turn the kicker
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *   - genevaState:
 *       Type: Int
 *       Descr: The state in which the Geneva Drive should go to. -2 means leftmost state, 0 means 'neutral' state, 2 means rightmost state.
 */
class Geneva : public Skill {
public:
	Geneva(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize() override;
	Status Update() override;

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
		params["genevaState"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() override { return "Geneva"; }
private:
	int robotID;
	int genevaState;

    int cycleCounter;
};


} // rtt
