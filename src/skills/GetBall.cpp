#include "ros/ros.h"

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/GetBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"

#include <cmath>
#include <vector>
#include <string>

#define RTT_CURRENT_DEBUG_TAG GetBall

namespace rtt {

RTT_REGISTER_SKILL(GetBall);

GetBall::GetBall(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) {
    hasBall = whichRobotHasBall();
}

int GetBall::whichRobotHasBall() {
    auto holder = getBallHolder();
    if (!holder) {
        return -1;
    }
    our_team = holder->second;
    return holder->first.id;
}

void GetBall::publishStopCommand() {
	roboteam_msgs::RobotCommand command;
	command.id = robotID;
	command.x_vel = 0.0;
	command.y_vel = 0.0;
	command.w = 0.0;
	command.dribbler = true;

	auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}

void GetBall::Initialize() {
    ballCloseFrameCount = 0;
}

bt::Node::Status GetBall::Update (){
	roboteam_msgs::World world = LastWorld::get();
	robotID = blackboard->GetInt("ROBOT_ID");
	if (HasDouble("acceptableDeviation")) {
		acceptableDeviation = GetDouble("acceptableDeviation");
	}


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
        ROS_WARN_STREAM("GetBall: robot with this ID not found, ID: " << robotID);
    }  


	// Store some info about the world state
	roboteam_msgs::WorldBall ball = world.ball;
	Vector2 ballPos(ball.pos);
	Vector2 ballVel(ball.vel);
	Vector2 robotPos(robot.pos);
	Vector2 robotVel(robot.vel);
	Vector2 targetPos;
	double targetAngle;

	Vector2 interceptPos = ballPos; // + ballVel.scale(0.25);

	// If we need to face a certain direction directly after we got the ball, it is specified here. Else we just face towards the ball
	if (HasString("AimAt")) {
		targetAngle = GetTargetAngle(ballPos, GetString("AimAt"), GetInt("AimAtRobot"), GetBool("AimAtRobotOurTeam")); // in roboteam_tactics/utils/utils.cpp
		// ROS_INFO_STREAM("targetAngle: " << targetAngle);
		// targetAngle = (Vector2(-4.5, 0.0) - robotPos).angle();
	} else {
		if (HasDouble("targetAngle")) {
			targetAngle = GetDouble("targetAngle");
		} else {
			targetAngle = (interceptPos - robotPos).angle();
		}
	}
	targetAngle = cleanAngle(targetAngle);

	// Limit the difference between the targetAngle and the direction we're driving towards to 90 degrees so we don't hit the ball
	// This is no problem, because the direction we're driving towards slowly converges to the targetAngle as we drive towards the 
	// target position. It's hard to explain without drawing, for questions ask Jim :)
	double angleDiff = (targetAngle - (interceptPos - robotPos).angle());
	angleDiff = cleanAngle(angleDiff);
	if (angleDiff > 0.5*M_PI) {
		targetAngle = (interceptPos - robotPos).angle() + 0.5*M_PI;
	}
	if (angleDiff < -0.5*M_PI) {
		targetAngle = (interceptPos - robotPos).angle() - 0.5*M_PI;
	}
	

	// Only once we get close enough to the ball, our target position is one directly touching the ball. Otherwise our target position is 
	// at a distance of 25 cm of the ball, because that allows for easy rotation around the ball and smooth driving towards the ball.
	double posDiff = (interceptPos - robotPos).length();
    bool avoidBall = false;
	if (posDiff > 0.4 || fabs(angleDiff) > 0.2*M_PI) {
		targetPos = interceptPos + Vector2(0.3, 0.0).rotate(targetAngle + M_PI);
        avoidBall = true;
	} else {
		targetPos = interceptPos + Vector2(0.08, 0.0).rotate(targetAngle + M_PI);
        avoidBall = false;
	}


	// Check the IHaveBall condition to see whether the GetBall skill succeeded
	auto bb2 = std::make_shared<bt::Blackboard>();
	bb2->SetInt("me", robotID);
	bb2->SetBool("our_team", true);
	IHaveBall iHaveBall("", bb2);

	bt::Node::Status stat = iHaveBall.Update();

    if (stat == Status::Success) {
        ballCloseFrameCount++;
    } else {
        ballCloseFrameCount = 0;
    }

    // Only stop if ball has been here 5 frames
	if (stat == Status::Success 
            && fabs(targetAngle - robot.angle) < 0.1
            // Only send succes if either:
            //  - The ball was close for 8 or more frames
            //  - The ball must be kicked as soon as there's a chance of kicking it
            && (ballCloseFrameCount >= 8 || GetBool("passOn"))) {

		// Ideally we want to use the kick skill here, but it is the question whether that is fast enough to respond
		// in the situation when the ball is rolling and we are catching up
        roboteam_msgs::RobotCommand command;
        command.id = robotID;
        command.kicker = GetBool("passOn");
        command.kicker_forced = GetBool("passOn");
        command.kicker_vel = GetBool("passOn") ? 5.0 : 0;

        command.x_vel = 0;
        command.y_vel = 0;
        command.dribbler = true;

        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	    pub.publish(command);	

		RTT_DEBUGLN("GetBall skill completed.");
		return Status::Success;
	} else {
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        private_bb->SetBool("avoidRobots", true);
        // private_bb->SetBool("avoidBall", avoidBall);
        private_bb->SetBool("dribbler", false);
        private_bb->SetString("whichBot", GetString("whichBot"));
        // @HACK for robot testing purposes
        // if (HasDouble("maxSpeed")) {
        // 	private_bb->SetDouble("maxSpeed", GetDouble("maxSpeed"));
        // }
        // if (HasDouble("minAngularVel")) {
        // 	private_bb->SetDouble("minAngularVel", GetDouble("minAngularVel"));
        // }
        // if (HasDouble("pGainRotation")) {
        // 	private_bb->SetDouble("pGainRotation", GetDouble("pGainRotation"));
        // }
        


        boost::optional<roboteam_msgs::RobotCommand> commandPtr = goToPos.getVelCommand();
        roboteam_msgs::RobotCommand command;
        if (commandPtr) {
        	command = *commandPtr;
        } else {
        	ROS_WARN("GoToPos returned an empty command message! Maybe we are already there :O");
        }
        // ROS_INFO("command for %d: %f, %f, %f", command.id, command.x_vel, command.y_vel, command.w);

        // TODO: Commented this out because it was giving problems. Hopefully we can
        // activate it at some point.
        // Vector2 ballVelInRobotFrame = worldToRobotFrame(ballVel, robot.angle).scale(1.0);
        // Vector2 newVelCommand(command.x_vel + ballVelInRobotFrame.x, command.y_vel + ballVelInRobotFrame.y);
        // if (newVelCommand.length() > 2.0) {
            // newVelCommand.scale(2.0 / newVelCommand.length());
        // }
        // command.x_vel = newVelCommand.x;
        // command.y_vel = newVelCommand.y;

        // Get global robot command publisher, and publish the command
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	    pub.publish(command);	

		return Status::Running;
	}
}

} // rtt
