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

class GoToPos : public Skill {
public:
    GoToPos(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    void sendStopCommand(uint id);
    roboteam_utils::Vector2 positionController(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 targetPos);
    double rotationController(double myAngle, double angleGoal, roboteam_utils::Vector2 posError);
    roboteam_utils::Vector2 velocityController(roboteam_utils::Vector2 velTarget);
    double angularVelController(double angularVelTarget);
    roboteam_utils::Vector2 avoidRobots(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 targetPos);
    roboteam_utils::Vector2 getForceVectorFromRobot(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 otherRobotPos, roboteam_utils::Vector2 posError);
    roboteam_utils::Vector2 checkTargetPos(roboteam_utils::Vector2 targetPos);
    Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["xGoal"] = BBArgumentType::Double;
        params["yGoal"] = BBArgumentType::Double;
        return params;
    }
    std::string node_name() { return "GoToPos"; }
    
private:
    bool success;

    // Control gains
    double pGainPosition = 3.0;
    double pGainRotation = 6.0;
    double maxAngularVel = 3.0;
    double iGainVelocity = 0.2;
    double iGainAngularVel = 0.02;

    // Control variables
    double safetyMarginGoalAreas = 0.2;
    double marginOutsideField = 0.2;
    double maxSpeed;
    double attractiveForce;
    double attractiveForceWhenClose;
    double repulsiveForce;

    double xGoal;
    double yGoal;
    double angleGoal;
    uint   ROBOT_ID;
    bool   dribbler;
    roboteam_msgs::WorldRobot me;

    roboteam_utils::Vector2 prevTargetPos;
    roboteam_utils::Vector2 velControllerI;
    double angularVelControllerI;
    // double wControllerI;

    Draw drawer;
};

} // rtt
