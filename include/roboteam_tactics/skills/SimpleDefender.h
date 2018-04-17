#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"

#include "roboteam_tactics/skills/ReceiveBall.h"
#include "roboteam_tactics/skills/GetBall.h"


namespace rtt {

/**
 * \class SimpleDefender
 * \brief See YAML
 */
/*
 * Descr: Simple Keeper
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *
 *   - ourSide:
 *       Type: Bool
 *       Descr: Set to true if our goal should be defended, false if the opponent's goal (only for testing purposes), defaults to true
 *
 *   - fieldType:
 *       Type: String
 *       Descr: Used to specify the dimensions of the field
 *       Can be:
 *          - office: If used on the field in the office
 *          - default: SSL specifications
 */

class SimpleDefender : public Skill {
public:
	SimpleDefender(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    static Vector2 computeDefensePoint(Vector2 defendPos, bool ourSide, double distanceFromGoal, double angleOffset);
	Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }

    std::string node_name() override { return "SimpleDefender"; }
private:
    
	int robotID;
    ReceiveBall receiveBall;
    GoToPos goToPos;
    // double distanceFromGoal;
    // bool ourSide;
    Draw drawer;
};

} // rtt
