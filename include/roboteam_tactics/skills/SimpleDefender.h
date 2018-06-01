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
 *
 *
 *   - targetFromType / targetToType
 *      Type: String
 *      Descr: Used to specify the type of the FromTarget or the ToTarget
 *      Can be:
 *          - position: if the target is a fixed position
 *          - object: if the target is a moving object (ball, robot)
 *
 *
 *   - targetFromTypeX / targetToTypeX
 *      Type: Double
 *      Descr: x coordinate of the FromTarget or the ToTarget, used when targetType is position
 *
 *   - targetFromTypeY / targetToTypeY
 *      Type: Double
 *      Descr: y coordinate of the FromTarget or the ToTarget, used when targetType is position
 *
 *
 *   - targetFromObj / targetToObj
 *      Type: String
 *      Descr: Used to specify the kind of object, used when targetType is object
 *      Can be:
 *          - ball: if the position of the target is the position of the ball
 *          - us: if the position of the target is the position of on of our robots
 *          - them: if the position of the target is the position of on of their robots
 *
 *   - targetFromRobotId / targetToRobotId
 *      Type: Int
 *      Descr: used to specify the id of the robot, used when the targetType is robot
 */

class SimpleDefender : public Skill {
public:
	SimpleDefender(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    static Vector2 computeDefensePoint(Vector2 defendPos, bool ourSide, double distanceFromGoal, double angleOffset);
    static Vector2 computeDefensePointRatio(Vector2 targetFrom, Vector2 targetTo, double ratio);
    static Vector2 computeDefensePointAbsolute(Vector2 targetFrom, Vector2 targetTo, double distance);
    Vector2 getTargetFromPosition();
    Vector2 getTargetToPosition();
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

    // double distanceFromGoal;
    // bool ourSide;
    Draw drawer;
};

} // rtt
