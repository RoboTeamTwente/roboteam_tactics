#pragma once

#include <vector>
#include <boost/optional.hpp>

#include "ros/ros.h"

#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_utils/Cone.h"

namespace rtt {

/**
 * # Using the YAML multiline literal here
 * Descr: |
 *     Goes to a position
 *
 *     @Idea isKeeper should maybe also be global?
 *     @Idea same goes for canIGoIntoGoalArea & stay50cmAwayFromBall?
 *
 * Global params:
 *     ROBOT_ID:
 *         Type: Int 
 *         Descr: Id of the robot
 * 
 * Params:
 *     isKeeper:
 *         Type: Bool
 *         Descr: Whether or not the current robot is a keeper
 *     
 *     angleGoal:
 *         Type: Double
 *         Descr: The angle of the arrival position
 *     
 *     xGoal:
 *         Type: Double
 *         Descr: The target x coordinate
 *     
 *     yGoal:
 *         Type: Double
 *         Descr: The target y coordinate
 *     
 *     dribbler:
 *         Type: Bool
 *         Descr: Turns on the dribbler if true
 *
 *     avoidRobots:
 *         Type: Bool
 *         Descr: Indicates whether or not other robots should be avoided
 *
 */
class GoToPos : public Skill {
public:
    GoToPos(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    void sendStopCommand(uint id);
    roboteam_utils::Vector2 positionController(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 targetPos);
    double rotationController(double myAngle, double angleGoal, roboteam_utils::Vector2 posError);
    roboteam_utils::Vector2 velocityController(roboteam_utils::Vector2 velTarget);
    double angularVelController(double angularVelTarget);
    roboteam_utils::Vector2 getForceVectorFromRobot(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 otherRobotPos, double lookingDistance, Cone antennaCone);
    double getAngleFromRobot(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 otherRobotPos, double lookingDistance, Cone antennaCone);
    
    roboteam_utils::Vector2 avoidRobots(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 myVel, roboteam_utils::Vector2 targetPos);
    double avoidRobotsForward(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 myVel, roboteam_utils::Vector2 targetPos);
    
    roboteam_utils::Vector2 avoidDefenseAreas(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 myVel, roboteam_utils::Vector2 targetPos, roboteam_utils::Vector2 sumOfForces);
    roboteam_utils::Vector2 checkTargetPos(roboteam_utils::Vector2 targetPos);
    roboteam_msgs::RobotCommand getVelCommand();
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
    double pGainRotation = 1.0;
    // double iGainRotation = 50.0;
    double maxAngularVel = 3.0;
    double iGainVelocity = 0.5;
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
    uint   KEEPER_ID = 10;
    bool   dribbler;
    roboteam_msgs::WorldRobot me;

    roboteam_utils::Vector2 prevTargetPos;
    roboteam_utils::Vector2 velControllerI;
    double angularVelControllerI;
    // double rotationControllerI = 0;
    // double wControllerI;

    // bool hasReachedFirstStop = false;

    Draw drawer;
};

} // rtt
