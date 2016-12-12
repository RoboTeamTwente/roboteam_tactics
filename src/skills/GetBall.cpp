#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/Math.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/skills/GetBall.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#include <cmath>
#include <vector>
#include <string>

#define RTT_CURRENT_DEBUG_TAG GetBall

namespace rtt {

RTT_REGISTER_SKILL(GetBall);

GetBall::GetBall(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , avoidRobots("", private_bb) {
    hasBall = whichRobotHasBall();
    set_PARAM_KICKING(false);
}

int GetBall::whichRobotHasBall() {
    auto holder = getBallHolder();
    if (!holder) {
        return -1;
    }
    our_team = holder->second;
    return holder->first.id;
}

InterceptPose GetBall::GetInterceptPos(double getBallAtX, double getBallAtY, double getBallAtTime) {
	InterceptPose interceptPose;
	roboteam_utils::Vector2 interceptPos;
	double interceptAngle;

	roboteam_msgs::World world = LastWorld::get();

	// Check if the same robot still has the ball
	auto bb2 = std::make_shared<bt::Blackboard>();
	bb2->SetInt("me", hasBall);
	bb2->SetBool("our_team", our_team);
	IHaveBall iHaveBall("", bb2);
	if (iHaveBall.Update() != Status::Success) {
		hasBall = whichRobotHasBall();
	}

	if (hasBall == -1) { // no robot has the ball

		// Predict intercept pos from ball velocity
		roboteam_msgs::Vector2f predictedBallPos = LastWorld::PredictBallPos(getBallAtTime);
		roboteam_utils::Vector2 ballPosNow = roboteam_utils::Vector2(world.ball.pos.x, world.ball.pos.y);
		roboteam_utils::Vector2 ballPosThen = roboteam_utils::Vector2(predictedBallPos.x, predictedBallPos.y);
		roboteam_utils::Vector2 getBallAtPos = roboteam_utils::Vector2(getBallAtX, getBallAtY);
		roboteam_utils::Vector2 ballTrajectory = ballPosThen - ballPosNow;
		roboteam_utils::Vector2 ballToCenter = getBallAtPos - ballPosNow;

		double ballTrajectoryMagn = ballTrajectory.length();
		ballTrajectory = ballTrajectory.scale(1/ballTrajectoryMagn);
		double projectionOnBallTrajectory = ballToCenter.dot(ballTrajectory);
		roboteam_utils::Vector2 closestPoint = ballPosNow + ballTrajectory*projectionOnBallTrajectory;

		if (ballTrajectoryMagn > 0.1 && (closestPoint - getBallAtPos).length() < acceptableDeviation) {
			interceptPos = closestPoint;
			interceptAngle = (ballPosNow-ballPosThen).angle();
		} else {
			interceptPos.x = getBallAtX;
			interceptPos.y = getBallAtY;
			interceptAngle = (ballPosNow - getBallAtPos).angle();
		}
		interceptPose.interceptPos = interceptPos;
		interceptPose.interceptAngle = interceptAngle;
		
		return interceptPose;

	} else {

		// Predict intercept pos by looking at the robot that has the bal
		roboteam_msgs::WorldRobot otherRobot;
		if (our_team) {
			otherRobot = world.us.at(hasBall);
		} else {
			otherRobot = world.them.at(hasBall);
		}

		roboteam_utils::Vector2 otherRobotLooksAt = roboteam_utils::Vector2(1, 0);
		otherRobotLooksAt = otherRobotLooksAt.rotate(otherRobot.angle);
		roboteam_utils::Vector2 ballPosNow = roboteam_utils::Vector2(world.ball.pos.x, world.ball.pos.y);
		roboteam_utils::Vector2 getBallAtPos = roboteam_utils::Vector2(getBallAtX, getBallAtY);
		roboteam_utils::Vector2 ballToCenter = getBallAtPos - ballPosNow;
		double projectionOnBallTrajectory = ballToCenter.dot(otherRobotLooksAt);
		roboteam_utils::Vector2 closestPoint = ballPosNow + otherRobotLooksAt*projectionOnBallTrajectory;

		if ((closestPoint - getBallAtPos).length() < acceptableDeviation) {
			interceptPos = closestPoint;
			if (otherRobot.angle > 0) {
				interceptAngle = otherRobot.angle - M_PI;
			} else {
				interceptAngle = otherRobot.angle + M_PI;
			}
		} else {
			interceptPos.x = getBallAtX;
			interceptPos.y = getBallAtY;
			interceptAngle = (ballPosNow - getBallAtPos).angle();
		}
		interceptPose.interceptPos = interceptPos; 
		interceptPose.interceptAngle = interceptAngle;
		
		return interceptPose;
	}
}

bool GetBall::IsPointInCircle(roboteam_utils::Vector2 center, double radius, roboteam_utils::Vector2 point) {
	double xDiffSqr = (point.x-center.x)*(point.x-center.x);
	double yDiffSqr = (point.y-center.y)*(point.y-center.y);
	double radiusSqr = radius*radius;
	return ((xDiffSqr + yDiffSqr) < (radiusSqr));
}

int GetBall::GetSign(double number) {
	if (number >= 0.0) {return 1;}
	if (number < 0.0) {return -1;}
	return 0;
}

bt::Node::Status GetBall::Update (){
	roboteam_msgs::World world = LastWorld::get();
	int robotID = blackboard->GetInt("ROBOT_ID");
	
	while (world.us.size() == 0) {
		return Status::Running;
	}

	roboteam_msgs::WorldBall ball = world.ball;
	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	double distanceToBall = (ballPos-robotPos).length();
	bool intercept = GetBool("intercept");
	roboteam_utils::Vector2 targetPos;
	double targetAngle;

	if (intercept) {
		if (distanceToBall > acceptableDeviation) {
			double getBallAtX;
			double getBallAtY;
			if (GetBool("getBallAtCurrentPos")) {
				getBallAtX = robotPos.x;
				getBallAtY = robotPos.y;
			} else {
				getBallAtX = GetDouble("getBallAtX");
				getBallAtY = GetDouble("getBallAtY");
			}
			
			double getBallAtTime = GetDouble("getBallAtTime");

			InterceptPose interceptPose = GetInterceptPos(getBallAtX, getBallAtY, getBallAtTime);
			roboteam_utils::Vector2 interceptPos = interceptPose.interceptPos;
			double interceptAngle = interceptPose.interceptAngle;

			roboteam_utils::Vector2 getBallAtPos = roboteam_utils::Vector2(getBallAtX, getBallAtY);
			if (IsPointInCircle(getBallAtPos, acceptableDeviation, interceptPos)) {
				targetPos = interceptPos;
				targetAngle = interceptAngle;
			} else {
				ROS_ERROR("This is probably not a valid intercept position...");
				return Status::Failure;
			}
				
		} else { // If we are close enough to the ball we can drive towards it and turn on the dribbler
			roboteam_utils::Vector2 posDiff = ballPos - robotPos;
			roboteam_utils::Vector2 posDiffNorm = posDiff.normalize();
			targetPos = ballPos - posDiffNorm.scale(0.09); // 0.09 = robot radius
			targetAngle = posDiff.angle();
			private_bb->SetBool("dribbler", true);
		}
	} else { // If we need not intercept the ball, then just drive towards it
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
		double posDiff = (ballPos - robotPos).length();

		if (posDiff > 0.3 || fabs(angleDiff) > 0.2*M_PI) {
			targetPos = ballPos + roboteam_utils::Vector2(0.2, 0.0).rotate(targetAngle + M_PI);
		} else {
			targetPos = ballPos + roboteam_utils::Vector2(0.09, 0.0).rotate(targetAngle + M_PI);
		}
	}

	auto bb2 = std::make_shared<bt::Blackboard>();
	bb2->SetInt("me", robotID);
	bb2->SetBool("our_team", true);
	IHaveBall iHaveBall("", bb2);

	if (iHaveBall.Update() == Status::Success && fabs(targetAngle - robot.angle) < 0.05) {
		roboteam_msgs::RobotCommand command;
		command.id = robotID;
		command.x_vel = 0.0;
		command.y_vel = 0.0;
		command.w = 0.0;
		command.dribbler = true;
        rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
		ros::spinOnce();

		RTT_DEBUG("GetBall skill completed.");
		return Status::Success;
	} else {
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        
        avoidRobots.Update();
		prevTargetPos = roboteam_utils::Vector2(targetPos.x, targetPos.y);
		prevTargetAngle = targetAngle;
		return Status::Running;
	}
};

} // rtt
