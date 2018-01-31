#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ros/ros.h>
#include <cmath>
#include <boost/optional.hpp>


#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/BallPlacementTest.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/conditions/IsBallInDefenseArea.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"




#define RTT_CURRENT_DEBUG_TAG BallPlacementTest

namespace rtt {

RTT_REGISTER_SKILL(BallPlacementTest);

BallPlacementTest::BallPlacementTest(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPosObj("", private_bb)

        {
            succeeded = false;
            failure = false;
            ros::NodeHandle n;
            myPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myPosTopic", 1000);
            myVelTopic = n.advertise<roboteam_msgs::WorldRobot>("myVelTopic", 1000);
            myTargetPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myTargetPosTopic", 1000);
            controller.Initialize(blackboard->GetInt("ROBOT_ID"));


            safetyMarginGoalAreas = 0.2;
            marginOutsideField = 0.3;
            avoidRobotsGain = 0.15;

        }


    void BallPlacementTest::publishStopCommand() {
        roboteam_msgs::RobotCommand command;
        command.id = robotID;
        command.x_vel = 0.0;
        command.y_vel = 0.0;
        command.w = 0.0;
        command.dribbler = false;

        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        pub.publish(command);
    }


void BallPlacementTest::sendStopCommand(uint id) {
    roboteam_msgs::RobotCommand command = controller.getStopCommand(id);

    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}

    bt::Node::Status BallPlacementTest::Update (){

        roboteam_msgs::World world = LastWorld::get();
        robotID = blackboard->GetInt("ROBOT_ID");


        // Wait for the first world message
        while (world.us.size() == 0) {
            return Status::Running;
        }


        // Find the robot with the specified ID
        boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
        roboteam_msgs::WorldRobot robot;
        if (findBot) {
            robot = *findBot;
        } else {
            ROS_WARN_STREAM("BallPlacementTest: robot with this ID not found, ID: " << robotID);
            return Status::Failure;
        }

        // Store some info about the world state
        roboteam_msgs::WorldBall ball = world.ball;
        Vector2 robotPos(robot.pos);
        Vector2 robotVel(robot.vel);

        // get position for ball placement
        Vector2 ballPlacePosition = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
        std::cout << "xGoal: " << ballPlacePosition.x << ", yGoal: " << ballPlacePosition.y << std::endl;

        // set target position (targetPos) for the robot
        Vector2 ballPos = Vector2(ball.pos.x, ball.pos.y);
        Vector2 ballPosError = ballPlacePosition - ballPos;
        Vector2 targetPos = robotPos + ballPosError;

        // set target angle for the robot
        double targetAngle;
        targetAngle = ballPosError.angle() + M_PI;

        // to see if the robot lost the ball
        Vector2 robotBallError = robotPos - ballPos;
        if (robotBallError.length() > 0.5){
            ROS_WARN_STREAM("BallPlacementTest: robot " << robotID << " lost ball");
            sendStopCommand(robotID);
            failure = true;
            succeeded = false;
            return Status::Failure;
        }


        if (ballPosError.length() < 0.08){


            if(!!HasInt("counter")){

                blackboard->SetInt("counter", 0);
                ROS_WARN_STREAM("counter set to " << blackboard->GetInt("counter"));
            }

            if (blackboard->GetInt("counter") < 5){
                int counterSafe = blackboard->GetInt("counter");
                counterSafe = counterSafe + 1;

                blackboard->SetInt("counter", counterSafe);
                ROS_WARN_STREAM("counter++ " << counterSafe);
                return Status::Running;
            }

            else {
                ROS_WARN_STREAM("BallPlacementTest: !!!!!!!!!!!!!!!!!!!!!!ball placement succeeded!!!!!!!!!!!!!!!!!!!!!!");
                sendStopCommand(robotID);
                failure = false;
                succeeded = true;
                return Status::Success;
            }
        }


//        // Different !!GetBall!! parameters for grsim than for real robots
//        // !!so needs to be fine tuned for the ball placement!!
//        std::string robot_output_target = "";
//        ros::param::getCached("robot_output_target", robot_output_target);
//        double successDist;
//        double successAngle;
//        double getBallDist;
//        double distAwayFromBall;
//        if (robot_output_target == "grsim") {
//            successDist = 0.11;
//            successAngle = 0.2;
//            getBallDist = 0.09 ;
//            distAwayFromBall = 0.2;;
//        } else if (robot_output_target == "serial") {
//            successDist = 0.115 ;
//            successAngle = 0.15;
//            getBallDist = 0.05;
//            distAwayFromBall = 0.2;
//        }



        // turn on dribbler
        private_bb->SetBool("dribbler", true);
        std::cout << "dribbler on" << std::endl;



//        // Return Success if we've been close to the ball for a certain number of frames
//        // !!Needs to be turned into if we've been close to the ball placement location!!
//        double angleError = cleanAngle(robot.angle - targetAngle);
//        if ((ballPos - robotPos).length() < successDist && fabs(angleError) < successAngle) {
//            // matchBallVel = false;
//            int ballCloseFrameCountTo = 10;
//            if(HaFInt("ballCloseFrameCount")){
//                ballCloseFrameCountTo = GetInt("ballCloseFrameCount");
//            }
//
//            if (ballCloseFrameCount < ballCloseFrameCountTo) {
//                ballCloseFrameCount++;
//                return Status::Running;
//            } else {
//
//                if (choseRobotToPassTo) {
//                    publishKickCommand(3.0);
//                } else {
//                    publishKickCommand(8.0);
//                }
//
//                if(countFinalMessages < 10){
//                    countFinalMessages=countFinalMessages+1;
//                    return Status::Running;
//                }
//                else {
//                    choseRobotToPassTo = false;
//                    releaseBall();
//                    return Status::Success;
//                }
//
//                return Status::Running;
//            }
//        } else {
//            ballCloseFrameCount = 0;
//        }


        // Set the blackboard for GoToPos
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
        private_bb->SetDouble("maxSpeed", 1);
        private_bb->SetDouble("successDist", 0.08);
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        private_bb->SetBool("avoidRobots", false);
        if (HasBool("enterDefenseAreas")) {
            private_bb->SetBool("enterDefenseAreas", GetBool("enterDefenseAreas"));
        }

        if (HasDouble("pGainPosition")) {
            private_bb->SetDouble("pGainPosition", GetDouble("pGainPosition"));
        }
        if (HasDouble("pGainRotation")) {
            private_bb->SetDouble("pGainRotation", GetDouble("pGainRotation"));
        }

        // Get the velocity command from GoToPos
        boost::optional<roboteam_msgs::RobotCommand> commandPtr = goToPosObj.getVelCommand();
        roboteam_msgs::RobotCommand command;
        if (commandPtr) {
            command = *commandPtr;
            command.w = 0;

            // Get global robot command publisher, and publish the command
            auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
            pub.publish(command);

            return Status::Running;
        } else {
            publishStopCommand();
            return Status::Running;
        }



    }


} // rtt
