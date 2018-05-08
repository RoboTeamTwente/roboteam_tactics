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
#include "roboteam_tactics/conditions/IsInDefenseArea.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"




#define RTT_CURRENT_DEBUG_TAG BallPlacementTest

namespace rtt {

RTT_REGISTER_SKILL(BallPlacementTest);

BallPlacementTest::BallPlacementTest(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPosObj("", private_bb){

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
        Vector2 ballPlacePosition = Vector2(GetDouble("xPlace"), GetDouble("yPlace"));
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

        // test if the ball is within the success distance for 5 frames. if so, return success
        if (ballPosError.length() < 0.08){

            if (blackboard->GetInt("counter") < 0){
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



        // Set the blackboard for GoToPos
        private_bb->SetInt("ROBOT_ID", robotID);                            // sets robot id
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));   // sets keeper id
        private_bb->SetBool("dribbler", true);                              // turn on dribbler
        private_bb->SetDouble("maxSpeed", 0.5);                             // sets maximum speed (which is low for ball placement)
        private_bb->SetDouble("successDist", 0.08);                         // sets succes distance
        private_bb->SetDouble("xGoal", targetPos.x);                        // x location for ball placement
        private_bb->SetDouble("yGoal", targetPos.y);                        // y location for ball placement
        private_bb->SetDouble("angleGoal", targetAngle);                    // final goal for the angle of the robot
        private_bb->SetBool("avoidRobots", false);                          // the robot does not have to avoid robots during ball placement

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
            command.w = 0;          //this makes sure the robot will not turn during ball placement

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
