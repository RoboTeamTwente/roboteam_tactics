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
    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
}

// roboteam_utils::Vector2 GetBall::computeInterceptPoint(roboteam_utils::Vector2 currentPos, roboteam_utils::Vector2 currentVel) {

// 	auto bb3 = std::make_shared<bt::Blackboard>();
// 	CanReachPoint canReachPoint("", bb3);
// 	double estimatedTimeToPoint = 0.6;
// 	double testTime = 0.5;
// 	roboteam_utils::Vector2 interceptPos;

// 	while (estimatedTimeToPoint > testTime) {
// 		interceptPos = LastWorld::predictBallPos(testTime);
// 		double estimatedTimeToPoint = canReachPoint.estimateTimeToPoint(currentPos, currentVel, interceptPos);
// 		testTime = testTime + 0.5;
// 	}

// 	ROS_INFO_STREAM("calculated intercept pos and found: " << interceptPos.x << " " << interceptPos.y);
// 	return interceptPos;
// }

bt::Node::Status GetBall::Update (){

	roboteam_msgs::World world = LastWorld::get();
	int robotID = blackboard->GetInt("ROBOT_ID");
	if (HasDouble("acceptableDeviation")) {
		acceptableDeviation = GetDouble("acceptableDeviation");
	}

	// Wait for the first world message
	while (world.us.size() == 0) {
		return Status::Running;
	}

	// Store some info about the world state
	roboteam_msgs::WorldBall ball = world.ball;
	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_utils::Vector2 ballPos(ball.pos);
	roboteam_utils::Vector2 ballVel(ball.vel);
	roboteam_utils::Vector2 robotPos(robot.pos);
	roboteam_utils::Vector2 robotVel(robot.vel);
	roboteam_utils::Vector2 targetPos;
	double targetAngle;


	// If we need to face a certain direction directly after we got the ball, it is specified here. Else we just face towards the ball
	if (HasString("AimAt")) {
		targetAngle = GetTargetAngle(robotID, true, GetString("AimAt"), GetInt("AimAtRobot"), GetBool("AimAtRobotOurTeam")); // in roboteam_tactics/utils/utils.cpp
	} else {
		targetAngle = (ballPos - robotPos).angle();
	}
	targetAngle = cleanAngle(targetAngle);
		

	// Limit the difference between the targetAngle and the direction we're driving towards to 90 degrees so we don't hit the ball
	// This is no problem, because the direction we're driving towards slowly converges to the targetAngle as we drive towards the 
	// target position. It's hard to explain without drawing, for questions ask Jim :)
	double angleDiff = (targetAngle - (ballPos - robotPos).angle());
	angleDiff = cleanAngle(angleDiff);

	if (angleDiff > 0.5*M_PI) {
		targetAngle = (ballPos - robotPos).rotate(0.5*M_PI).angle();
	} else if (angleDiff < -0.5*M_PI) {
		targetAngle = (ballPos - robotPos).rotate(-0.5*M_PI).angle();
	}
	

	// Only once we get close enough to the ball, our target position is one directly touching the ball. Otherwise our target position is 
	// at a distance of 20 cm of the ball, because that allows for easy rotation around the ball and smooth driving towards the ball.
	double posDiff = (ballPos - robotPos).length();
	if (posDiff > 0.3 || fabs(angleDiff) > 0.2*M_PI) {
		targetPos = ballPos + roboteam_utils::Vector2(0.2, 0.0).rotate(targetAngle + M_PI);
	} else {
		targetPos = ballPos + roboteam_utils::Vector2(0.06, 0.0).rotate(targetAngle + M_PI);
	}


	// Check the IHaveBall condition to see whether the GetBall skill succeeded
	auto bb2 = std::make_shared<bt::Blackboard>();
	bb2->SetInt("me", robotID);
	bb2->SetBool("our_team", true);
	IHaveBall iHaveBall("", bb2);

	double ballSpeed = Vector2(world.ball.vel.x, world.ball.vel.y).length();	

	if (iHaveBall.Update() == Status::Success && fabs(targetAngle - robot.angle) < 0.05 && ballSpeed < 0.1) {
		publishStopCommand();
		RTT_DEBUGLN("GetBall skill completed.");
		return Status::Success;
	} else {
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        private_bb->SetBool("avoidRobots", true);
        goToPos.Update();
		return Status::Running;
	}
}

} // rtt
