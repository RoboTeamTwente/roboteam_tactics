#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/skills/ReceiveBall.h"
#include "roboteam_tactics/skills/GetBall.h"

namespace rtt {

/**
 * \class Simple Keeper
 * \brief See YAML
 */
/*
 * Descr: Simple Keeper
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 */
class SimpleKeeper : public Skill {
public:
	SimpleKeeper(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }

    std::string node_name() override { return "SimpleKeeper"; }
private:
	int robotID;
    ReceiveBall receiveBall;
    GoToPos goToPos;
    
};


} // rtt
