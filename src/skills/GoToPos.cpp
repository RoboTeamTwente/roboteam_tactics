#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ros/ros.h>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/FieldLineSegment.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/Avoid_Areas_Objects.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/conditions/IsInDefenseArea.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Section.h"


#define RTT_CURRENT_DEBUG_TAG GoToPos
#define ROS_LOG_NAME "skills.GoToPos"

namespace rtt {

    //gets the robot to a certain position, it "should" choose it's orientation such that it get's there optmally
    //However as a result we cannot control the end angle, now it's dependant on the end-position relative to the
    //begin position.
    //It also calls functions that adjust the path, such that it avoids things like the robot, ball, and goal areas.
    RTT_REGISTER_SKILL(GoToPos);

    GoToPos::GoToPos(std::string name, bt::Blackboard::Ptr blackboard)
            : Skill(name, blackboard)

    {
        succeeded = false;
        failure = false;
        controller.Initialize(blackboard->GetInt("ROBOT_ID"));

        // Debug info
        ros::NodeHandle n;
        myPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myPosTopic", 1000);
        myVelTopic = n.advertise<roboteam_msgs::WorldRobot>("myVelTopic", 1000);
        myTargetPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myTargetPosTopic", 1000);

        //DEFAULTS
        ros::param::get("robot_output_target", robot_output_target);
        if (robot_output_target == "grsim") {
            grsim = true;
            safetyMarginGoalAreas = 0.1;
            marginOutsideField = 0.3;
            avoidRobotsGain = 1.0;
            cushionGain = 1.0;
            maxDistToAntenna = 0.3; // no force is exerted when distToAntenna is larger than maxDistToAntenna
        } else {
            grsim = false;
            safetyMarginGoalAreas = 0.1;
            marginOutsideField = 0.15;
            avoidRobotsGain = 1.0;
            cushionGain = 1.5;
            maxDistToAntenna = 0.3; // no force is exerted when distToAntenna is larger than maxDistToAntenna
        }

        //PROCESS BLACKBOARD
        if (HasDouble("avoidRobotsGain")) {
            avoidRobotsGain = GetDouble("avoidRobotsGain");
        }
        if (HasDouble("cushionGain")) {
            cushionGain = GetDouble("cushionGain");
        }
        if (HasDouble("maxDistToAntenna")) {
            maxDistToAntenna = GetDouble("maxDistToAntenna");
        }
    }


    void GoToPos::Initialize() {
        succeeded = false;
        failure = false;
        controller.setPresetControlParams();

        if (GetBool("driveBackward")) {
            ROBOT_ID = blackboard->GetInt("ROBOT_ID");
            // Get the latest world state
            roboteam_msgs::World world = LastWorld::get();
            // Find the robot with the specified ID
            boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(ROBOT_ID);
            roboteam_msgs::WorldRobot me;
            if (findBot) {
                me = *findBot;
            } else {
                return;
            }

            backwardPos = Vector2(me.pos) + Vector2(-0.3,0).rotate(me.angle);
        }
    }

    void GoToPos::sendStopCommand(uint id) {
        roboteam_msgs::RobotCommand command;
        command.id = ROBOT_ID;
        command.x_vel = 0.0;
        command.y_vel = 0.0;
        command.w = 0.0;
        command.dribbler = false;

        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        pub.publish(command);
    }


    boost::optional<roboteam_msgs::RobotCommand> GoToPos::getVelCommand() {

        ROBOT_ID = blackboard->GetInt("ROBOT_ID");
        KEEPER_ID = blackboard->GetInt("KEEPER_ID");
        Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));


        if (HasDouble("pGainPosition")) {
            controller.setControlParam("pGainPosition", GetDouble("pGainPosition"));
        }
        else if (HasBool("pGainLarger") && GetBool("pGainLarger") && !grsim) {
            controller.setControlParam("pGainPosition", 6.5);
            controller.setControlParam("dGainPosition", 1.3);
        }
        if (HasDouble("iGainPosition")) {
            controller.setControlParam("iGainPosition", GetDouble("iGainPosition"));
        }
        if (HasDouble("dGainPosition")) {
            controller.setControlParam("dGainPosition", GetDouble("dGainPosition"));
        }
        if (HasDouble("pGainRotation")) {
            controller.setControlParam("pGainRotation", GetDouble("pGainRotation"));
        }
        if (HasDouble("iGainRotation")) {
            controller.setControlParam("iGainRotation", GetDouble("iGainRotation"));
        }
        if (HasDouble("dGainRotation")) {
            controller.setControlParam("dGainRotation", GetDouble("dGainRotation"));
        }
        if (HasDouble("maxSpeed")) {
            controller.setControlParam("maxSpeed", GetDouble("maxSpeed"));
            // ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "maxSpeed set to : " << GetDouble("maxSpeed"));
        } else if (GetBool("lowSpeed")) {
            if (grsim) {
                controller.setControlParam("maxSpeed", 0.5);
            } else {
                controller.setControlParam("maxSpeed", 1.3);
            }
        }
        if (HasDouble("maxAngularVel")) {
            controller.setControlParam("maxAngularVel", GetDouble("maxAngularVel"));
        }
        if (HasDouble("thresholdTarget")) {
            controller.setControlParam("thresholdTarget", GetDouble("thresholdTarget"));
        }
        if (HasDouble("minTarget")) {
            controller.setControlParam("minTarget", GetDouble("minTarget"));
        }


        // Get the latest world state
        roboteam_msgs::World world = LastWorld::get();

        // Find the robot with the specified ID
        boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(ROBOT_ID, true, world);
        roboteam_msgs::WorldRobot me;
        if (findBot) {
            me = *findBot;
        } else {
            ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Robot with this ID not found, ID: " << ROBOT_ID);
            failure = true;
            succeeded = false;
            return boost::none;
        }

        // Store some variables for easy access
        Vector2 myPos(me.pos);
        Vector2 myVel(me.vel);

        // Determine angle goal and error
        double angleGoal;
        if (HasDouble("angleGoal")) {
            angleGoal = cleanAngle(GetDouble("angleGoal"));
        } else {
            angleGoal = me.angle;
        }

        double myAngle = me.angle;
        double angleError = cleanAngle(angleGoal - myAngle);
        double myAngularVel = me.w;

        // Determine how close we should get to the targetPos before we succeed
        double successDist;
        if (HasDouble("successDist")) {
            successDist = GetDouble("successDist");
        } else {
            successDist = 0.05;
        }

        // Check the input position
        if (targetPos == prevTargetPos) {
            targetPos = prevTargetPos;
        } else {
            targetPos = checkTargetPos(targetPos, myPos);
            prevTargetPos = targetPos;
        }

        if (HasBool("stayAwayFromBall") && GetBool("stayAwayFromBall")) {
            // ROS_INFO_STREAM("robot" << ROBOT_ID << " in stayAwayFromBall" );
            Vector2 ballPos(world.ball.pos);
            if ((ballPos - targetPos).length() < 0.7) {
                Vector2 diffVecNorm = (targetPos - ballPos).normalize();
                targetPos = ballPos + diffVecNorm.scale(0.7);
            }
        }

        Vector2 posError = targetPos - myPos;
        // If we are close enough to our target position and target orientation, then stop the robot and return success
        if (posError.length() < successDist && fabs(angleError) < 0.01) {
            successCounter++;
            if (successCounter >= 3) {
                succeeded = true;
                failure = false;
                return boost::none;
            }
        } else {
            successCounter = 0;
        }

        // Turn towards goal when error is too large
        static double posErrorRotationThreshold = 0.45;
        if (posError.length() > posErrorRotationThreshold && !GetBool("dontRotate")) {
            angleGoal = posError.angle();
            bool backwards = fabs(cleanAngle(angleGoal - myAngle)) > M_PI/2 && !GetBool("notBackwards");
            if (backwards) {
                angleGoal = cleanAngle(angleGoal + M_PI); // chooses either driving forward or backwards
            }
            angleError = cleanAngle(angleGoal - myAngle);
            posErrorRotationThreshold = 0.4;
        } else {
            posErrorRotationThreshold = 0.45;
        }

        // Draw the line towards the target position
        drawer.setColor(0, 100, 100);
        drawer.drawLine("posError_" + std::to_string(ROBOT_ID), myPos, posError);
        drawer.setColor(0, 0, 0);

        // A vector to combine all the influences of different controllers (normal position controller, obstacle avoidance, defense area avoidance...)
        Vector2 sumOfForces(0.0, 0.0);

        // Position controller to steer the robot towards the target position
        sumOfForces = sumOfForces + controller.positionController(myPos, targetPos, myVel);

        // Robot avoidance
        if (HasBool("avoidRobots") && !GetBool("avoidRobots")) {
            // AvoidRobots defaults to true if not set
        } else {
            sumOfForces = avoidRobots(myPos, myVel, targetPos, sumOfForces);
        }

        // Ball avoidance
        if (HasBool("avoidBall") && GetBool("avoidBall")) {
            Vector2 ballPos = Vector2(world.ball.pos);
            sumOfForces = sumOfForces + avoidBall(ballPos, myPos, sumOfForces, targetPos, myVel);
        }
        // Defense area avoidance
        if (!(HasBool("enterDefenseAreas") && GetBool("enterDefenseAreas"))) {
            sumOfForces = avoidDefenseAreas(myPos, targetPos, sumOfForces);
        }


        // Limit the command derivative (to apply a smoother command to the robot, which it can handle better)
        static time_point prevTime = now();
        int timeDiff = time_difference_milliseconds(prevTime, now()).count();
        prevTime = now();
        double max_diff = 20.0*timeDiff*0.001;
        static Vector2 prevCommand;
        Vector2 commandDiff = sumOfForces - prevCommand;
        if (commandDiff.length() > max_diff) {
            sumOfForces = (prevCommand + commandDiff.stretchToLength(max_diff));
        }
        prevCommand = sumOfForces;
        //////////

        // Draw the target velocity vector in rqt-view (in red)
        drawer.setColor(255, 0, 0);
        drawer.drawLine("velTarget" + std::to_string(ROBOT_ID), myPos, sumOfForces.scale(0.5));
        drawer.setColor(0, 0, 0);

        // Different velocity commands for grsim and real robot
        roboteam_msgs::RobotCommand command;
        Vector2 velCommand;
        if (grsim) {

            velCommand = worldToRobotFrame(sumOfForces, myAngle);   // Rotate the commands from world frame to robot frame
            double angularVelCommand = controller.rotationController(myAngle, angleGoal, posError, myAngularVel); // Rotation controller

            if ( HasBool("dontRotate") && GetBool("dontRotate") ) {
                command.w = 0;
                angleError = 0;
            } else {
                command.w = angularVelCommand;
            }

            velCommand = controller.limitVel2(velCommand, angleError);          // Limit linear velocity
            angularVelCommand = controller.limitAngularVel(angularVelCommand);  // Limit angular velocity

        } else {
            // // The rotation of linear velocity to robot frame happens on the robot itself now
            // // Also, the robot has its own rotation controller now. Make sure this is enabled on the robot
            if ( (HasBool("dontRotate") && GetBool("dontRotate"))) {
                command.use_angle = false;
                angleError = 0;
            } else {
                command.use_angle = true;
            }

            velCommand = sumOfForces; //worldToRobotFrame(sumOfForces, myAngle);   // Rotate the commands from world frame to robot frame
            velCommand = controller.limitVel2(velCommand, angleError);

            double angleCommand = angleGoal*16; // make sure it fits in the angularvel package
            command.w = angleCommand;
        }

        // Apply any max velocity that is set
        double maxVel = GetDouble("maxVelocity", 299792458.0);
        if (velCommand.length() > maxVel) {
            velCommand = velCommand.stretchToLength(maxVel);
        }

        // fill the rest of command message
        command.id = ROBOT_ID;
        command.dribbler = GetBool("dribbler");
        command.x_vel = velCommand.x;
        command.y_vel = velCommand.y;

        return command;
    }


    bt::Node::Status GoToPos::Update() {

        boost::optional<roboteam_msgs::RobotCommand> command = getVelCommand();
        if (command) {
            auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
            if (command->x_vel != command->x_vel) {
                ROS_ERROR("Tried to send NAN");
                return Status::Invalid;
            }
            pub.publish(*command);
            return Status::Running;
        } else {
            sendStopCommand(ROBOT_ID);

            if (succeeded) {
                return Status::Success;
            } else if (failure) {
                return Status::Failure;
            } else {
                return Status::Invalid;
            }
        }
    }

} // rtt
