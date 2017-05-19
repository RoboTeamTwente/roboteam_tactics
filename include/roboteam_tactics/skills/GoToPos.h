#pragma once

#include "ros/ros.h"

#include <vector>
#include <boost/optional.hpp>

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/Control.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_msgs/WorldRobot.h"


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
 * Params:
 *     - ROBOT_ID:
 *         Type: Int 
 *         Descr: Id of the robot
 *     - KEEPER_ID:
 *         Type: Int
 *         Descr: Id of the keeper (for defense area avoidance)
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
    Vector2 getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna, Vector2 targetPos);
    Vector2 avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos);    
    Vector2 avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces);
    Vector2 avoidBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces);
    Vector2 checkTargetPos(Vector2 targetPos);

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

    // Control parameters
    double avoidRobotsGain;
    Vector2 posErrorI;
    double angleErrorI;

    // Safety margins used to filter the target position
    double safetyMarginGoalAreas;
    double marginOutsideField;

    // Global blackboard info
    uint ROBOT_ID;
    uint KEEPER_ID;

    // Info about previous states
    Vector2 prevTargetPos;
    Vector2 prevSumOfForces;
    Vector2 prevVelCommand;
    double prevAngularVelTarget;
    
    // Success 
    int successCounter;
    bool succeeded;
    bool failure;

    Draw drawer;
    RobotType robotType;
    Control controller;

    // @DEBUG info
    ros::Publisher myPosTopic;
    ros::Publisher myVelTopic;
    ros::Publisher myTargetPosTopic;
} ;

} // rtt