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

#include "roboteam_tactics/conditions/IsInDefenseArea.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"


namespace rtt {

#define STOP_STATE_MAX_VELOCITY 1.3 // Actually 1.5 m/s, but there's no point in pushing the limit here.

/**
 * \class BallPlacement
 * \brief See YAML
 */
/**
 * # Using the YAML multiline literal here
 * Descr: Goes to a position
 *
 * Params:
 *    - ROBOT_ID:
 *        Type: Int
 *        Descr: Id of the robot
 *    - KEEPER_ID:
 *        Type: Int
 *        Descr: Id of the keeper (for defense area avoidance)
 *
 *    - angleGoal:
 *        Type: Double
 *        Descr: The angle of the arrival position
 *
 *    - xGoal:
 *        Type: Double
 *        Descr: The target x coordinate
 *
 *    - yGoal:
 *        Type: Double
 *        Descr: The target y coordinate
 *
 *    - dribbler:
 *        Type: Bool
 *        Descr: Turns on the dribbler if true
 *
 *    - avoidRobots:
 *        Type: Bool
 *        Descr: Indicates whether or not other robots should be avoided
 *
 *    - stayOnSide:
 *        Type: String
 *        Descr: Indicates on which side of the field GoToPos should stay.
 *        Can be:
 *           - ourSide: Have the robot stay at our side.
 *           - theirSide: Have the robot stay at their side.
 *           - TOTALLY EMPTY: Does nothing
 *
 *    - maxSpeed:
 *         Type: Double
 *         Descr: Sets the maximum robot speed in the controller
 *
 *    - enterDefenseAreas:
 *         Type: Bool
 *         Descr: When true allows GoToPos to go into the defense areas of both teams.
 *    - maxVelocity:
 *         Type: Double
 *         Descr: The maximum velocity the robot is allowed to achieve.
 *         Default: 299792458
 */
    class BallPlacement : public Skill {
    public:
        BallPlacement(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

        void sendStopCommand(uint id);
        Vector2 getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna, Vector2 targetPos);
        Vector2 avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos);

        boost::optional<roboteam_msgs::RobotCommand> getVelCommand();

        Status Update();

        void stoprobot(int RobotID);
        double computeAngle(Vector2 robotPos, Vector2 faceTowardsPos);

        double cleanAngle(double angle);
        Vector2 worldToRobotFrame(Vector2 requiredv, double rotation);
        Vector2 saveDribbleDeceleration(Vector2 reqspeed);

        static VerificationMap required_params() {
            VerificationMap params;
            params["ROBOT_ID"] = BBArgumentType::Int;
            params["xGoal"] = BBArgumentType::Double;
            params["yGoal"] = BBArgumentType::Double;
            return params;
        }
        std::string node_name() { return "BallPlacement"; }
//




    private:
        //      Dribble private:

        bool goal1reached=false;
        ros::NodeHandle n;
        ros::Publisher pubDebugpoints;
        int robotID;
        Vector2 robotvtogoal=Vector2(0.0,0.0);
        RotateAroundPoint rotateAroundPoint;
        Vector2 prevspeed=Vector2(0.0,0.0);
        roboteam_msgs::World::_header_type::_seq_type prevSeq;

        //      GoToPos private:

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
