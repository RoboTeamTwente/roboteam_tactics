#pragma once

#include <vector>
#include <boost/optional.hpp>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"

namespace rtt {

class AvoidRobots : public Skill {
public:
	AvoidRobots(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    roboteam_msgs::RobotCommand stopCommand(uint id);
    double RotationController(double angleError);
    // roboteam_msgs::RobotCommand PositionController(roboteam_utils::Vector2 posError, double angleError, double myAngle);
    roboteam_msgs::RobotCommand VelocityController(roboteam_utils::Vector2 velTarget, double wTarget, roboteam_utils::Vector2 posError);
    roboteam_utils::Vector2 GetForceVectorFromRobot(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 otherRobotPos, roboteam_utils::Vector2 posError);
    roboteam_utils::Vector2 CheckTargetPos(roboteam_utils::Vector2 targetPos);
    // roboteam_utils::Vector2 springDamperForce(roboteam_utils::Vector2 distToPoint, roboteam_utils::Vector2 attractionForce);
    Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["xGoal"] = BBArgumentType::Double;
        params["yGoal"] = BBArgumentType::Double;
        return params;
    }
    std::string node_name() { return "AvoidRobots"; }
    
private:
    bool success;

    // Control variables
    double maxSpeed;
    double attractiveForce;
    double attractiveForceWhenClose;
    double repulsiveForce;

    double xGoal;
    double yGoal;
    double angleGoal;
    uint   robotID;
    bool   dribbler;
    roboteam_msgs::WorldRobot me;

    roboteam_utils::Vector2 velControllerI;
    // double wControllerI;

    Draw drawer;
};

} // rtt
