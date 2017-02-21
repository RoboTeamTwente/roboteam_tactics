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

roboteam_utils::Vector2 GetBall::computeInterceptPoint(roboteam_utils::Vector2 currentPos, roboteam_utils::Vector2 currentVel) {

	auto bb3 = std::make_shared<bt::Blackboard>();
	CanReachPoint canReachPoint("", bb3);
	double estimatedTimeToPoint = 0.6;
	double testTime = 0.1;

	// roboteam_utils::Vector2 distanceToBall = 
	roboteam_utils::Vector2 interceptPos = LastWorld::predictBallPos(testTime);

	while (estimatedTimeToPoint > testTime) {
		
		double estimatedTimeToPoint = canReachPoint.estimateTimeToPoint(currentPos, currentVel, interceptPos);
		testTime = testTime + 0.1;
	}

	ROS_INFO_STREAM("calculated intercept pos and found: " << interceptPos.x << " " << interceptPos.y);
	return interceptPos;
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

	// Store some info about the world state
	roboteam_msgs::WorldBall ball = world.ball;
    // TODO: @Hack need lookupbot here!
	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_utils::Vector2 ballPos(ball.pos);
	roboteam_utils::Vector2 ballVel(ball.vel);
	roboteam_utils::Vector2 robotPos(robot.pos);
	roboteam_utils::Vector2 robotVel(robot.vel);
	roboteam_utils::Vector2 targetPos;
	double targetAngle;


	// double distanceToBall = (robotPos - ballPos).length();
	// roboteam_utils::Vector2 interceptPos = LastWorld::predictBallPos(distanceToBall*10);
	roboteam_utils::Vector2 interceptPos = ballPos + ballVel.scale(0.0);


	// If we need to face a certain direction directly after we got the ball, it is specified here. Else we just face towards the ball
	if (HasString("AimAt")) {
		targetAngle = GetTargetAngle(ballPos+ballVel.scale(1.25), GetString("AimAt"), GetInt("AimAtRobot"), GetBool("AimAtRobotOurTeam")); // in roboteam_tactics/utils/utils.cpp
		// targetAngle = (roboteam_utils::Vector2(-4.5, 0.0) - robotPos).angle();
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
		targetAngle = (interceptPos - robotPos).rotate(0.5*M_PI).angle();
	} else if (angleDiff < -0.5*M_PI) {
		targetAngle = (interceptPos - robotPos).rotate(-0.5*M_PI).angle();
	}

	// if (ballVel.length() > 0.5) {
	// 	targetAngle = cleanAngle(ballVel.angle() + M_PI);
	// }
	

	// Only once we get close enough to the ball, our target position is one directly touching the ball. Otherwise our target position is 
	// at a distance of 25 cm of the ball, because that allows for easy rotation around the ball and smooth driving towards the ball.
	double posDiff = (interceptPos - robotPos).length();
	if (posDiff > 0.4 || fabs(angleDiff) > 0.2*M_PI) {
		targetPos = interceptPos + roboteam_utils::Vector2(0.25, 0.0).rotate(targetAngle + M_PI);
	} else {
		targetPos = interceptPos + roboteam_utils::Vector2(0.08, 0.0).rotate(targetAngle + M_PI);
	}


	// Check the IHaveBall condition to see whether the GetBall skill succeeded
	auto bb2 = std::make_shared<bt::Blackboard>();
	bb2->SetInt("me", robotID);
	bb2->SetBool("our_team", true);
	IHaveBall iHaveBall("", bb2);

	// ROS_INFO_STREAM("targetAngle: " << targetAngle << " robotAngle: " << robot.angle);
	roboteam_utils::Vector2 aimDir = roboteam_utils::Vector2(1.0, 0.0).rotate(targetAngle);	
	drawer.DrawLine("aimDir", robotPos, robotPos+aimDir);
	// ROS_INFO_STREAM("angleError: " << targetAngle - robot.angle);

	bt::Node::Status stat = iHaveBall.Update();
	if (stat == Status::Success && fabs(targetAngle - robot.angle) < 0.1) {
		// publishStopCommand();

		if (GetBool("passOn")) {
			roboteam_msgs::RobotCommand command;
			command.id = robotID;
			command.kicker = true;
			command.kicker_forced = true;
			command.kicker_vel = 5.0;

			// Get global robot command publisher, and publish the command
	        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
		    pub.publish(command);	
		}

		publishStopCommand();
		RTT_DEBUGLN("GetBall skill completed.");
		return Status::Success;
	} else {

        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        private_bb->SetBool("avoidRobots", true);
        private_bb->SetBool("dribbler", false);
        roboteam_msgs::RobotCommand command = goToPos.getVelCommand();
        roboteam_utils::Vector2 ballVelInRobotFrame = worldToRobotFrame(ballVel, robot.angle).scale(2.0);
        roboteam_utils::Vector2 newVelCommand(command.x_vel + ballVelInRobotFrame.x, command.y_vel + ballVelInRobotFrame.y);
        if (newVelCommand.length() > 3.0) {
        	newVelCommand.scale(3.0 / newVelCommand.length());
        }
        command.x_vel = newVelCommand.x;
        command.y_vel = newVelCommand.y;


        // Get global robot command publisher, and publish the command
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	    pub.publish(command);	

		return Status::Running;
	}

	
}

} // rtt
