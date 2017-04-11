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

#include "roboteam_tactics/utils/utils.h"

namespace rtt {

/**
 * \class GoToPos
 * \brief See YAML
 */
/**
 * # Using the YAML multiline literal here
 * Descr: |
 *     Goes to a position
 *
 *     @Idea isKeeper should maybe also be global?
 *     @Idea same goes for canIGoIntoGoalArea & stay50cmAwayFromBall?
 *
 * Params:
 *     - ROBOT_ID:
 *         Type: Int 
 *         Descr: Id of the robot
 *     - isKeeper:
 *         Type: Bool
 *         Descr: Whether or not the current robot is a keeper
 *     
 *     - angleGoal:
 *         Type: Double
 *         Descr: The angle of the arrival position
 *     
 *     - xGoal:
 *         Type: Double
 *         Descr: The target x coordinate
 *     
 *     - yGoal:
 *         Type: Double
 *         Descr: The target y coordinate
 *     
 *     - dribbler:
 *         Type: Bool
 *         Descr: Turns on the dribbler if true
 *
 *     - avoidRobots:
 *         Type: Bool
 *         Descr: Indicates whether or not other robots should be avoided
 *
 *     - stayOnSide:
 *         Type: String
 *         Descr: Indicates on which side of the field GoToPos should stay.
 *         Can be:
 *             ourSide: Have the robot stay at our side.
 *             theirSide: Have the robot stay at their side.
 *             TOTALLY EMPTY: Does nothing
 *
 */
class GoToPos : public Skill {
public:
    GoToPos(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

    void sendStopCommand(uint id);

    Vector2 positionController(Vector2 myPos, Vector2 targetPos);
    double rotationController(double myAngle, double angleGoal, Vector2 posError);

    Vector2 getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, double lookingDistance, Cone antennaCone);
    Vector2 avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos);    
    Vector2 avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces);
    Vector2 avoidBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces);

    Vector2 checkTargetPos(Vector2 targetPos);
    Vector2 limitVel(Vector2 sumOfForces, Vector2 posError);
    double limitAngularVel(double angularVelTarget);

    boost::optional<roboteam_msgs::RobotCommand> getVelCommand();

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
    double pGainPosition;
    double pGainRotation;
    double maxAngularVel;
    double minAngularVel;

    // Control variables
    double maxSpeed;
    double minSpeedX;
    double minSpeedY;
    double safetyMarginGoalAreas;
    double marginOutsideField;

    // Blackboard arguments
    double xGoal;
    double yGoal;
    double angleGoal;
    uint   ROBOT_ID;
    uint   KEEPER_ID;
    bool   dribbler;
    roboteam_msgs::WorldRobot me;

    Vector2 prevTargetPos;
    Vector2 prevSumOfForces;
    double prevAngularVelTarget;
    bool succeeded;

    time_point start;

    Draw drawer;
};

} // rtt
