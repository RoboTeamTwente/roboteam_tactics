#pragma once

#include "ros/ros.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

/**
 * Chips the ball.
 *
 * Global params:
 *  - ROBOT_ID : Int 
 *    Id of the robot
 *
 */


class Chip : public Skill {
public:
    Chip(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize() override;
    Status Update() override;

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }

    std::string node_name() override { return "Chip"; }
private:
    int robotID;

    roboteam_utils::Vector2 oldBallVel;
    int cycleCounter;
};

} // rtt
