#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/LastWorld.h"
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

#include <cmath>
#include <vector>

namespace rtt {

GetBall::GetBall(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard)
        // , goToPos(n, "", private_bb)
        , avoidRobots(n, "", private_bb) {
    pubGetBall = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
    ROS_INFO("Getting ball");
    hasBall = whichRobotHasBall();
}

int GetBall::whichRobotHasBall() {
	roboteam_msgs::World world = LastWorld::get();
	for (size_t i=0; i < world.us.size(); i++) {
		auto bb2 = std::make_shared<bt::Blackboard>();
		bb2->SetInt("me", i);
		bb2->SetBool("our_team",blackboard->GetBool("our_team"));
		IHaveBall iHaveBall("", bb2);
		if (iHaveBall.Update() == Status::Success) {
			return i;
		}
	}
	for (size_t i=0; i < world.them.size(); i++) {
		auto bb2 = std::make_shared<bt::Blackboard>();
		bb2->SetInt("me", i);
		bb2->SetBool("our_team",blackboard->GetBool("our_team"));
		IHaveBall iHaveBall("", bb2);
		if (iHaveBall.Update() == Status::Success) {
			return (i+6);
		}
	}
	return -1;
}

InterceptPose GetBall::GetInterceptPos(double getBallAtX, double getBallAtY, double getBallAtTime) {
	InterceptPose interceptPose;
	roboteam_utils::Vector2 interceptPos;
	double interceptAngle;

	roboteam_msgs::World world = LastWorld::get();

	// Check if the same robot still has the ball
	auto bb2 = std::make_shared<bt::Blackboard>();
	bb2->SetInt("me", hasBall);
	bb2->SetBool("our_team",blackboard->GetBool("our_team"));
	IHaveBall iHaveBall("", bb2);
	if (iHaveBall.Update() != Status::Success) {
		hasBall = whichRobotHasBall();
	}
	// ROS_INFO_STREAM(hasBall);

	if (hasBall == -1) { // no robot has the ball
		// Predict intercept pos from ball velocity

		roboteam_msgs::Vector2f predictedBallPos = LastWorld::PredictBallPos(getBallAtTime);
		roboteam_utils::Vector2 ballPosNow = roboteam_utils::Vector2(world.ball.pos.x, world.ball.pos.y);
		roboteam_utils::Vector2 ballPosThen = roboteam_utils::Vector2(predictedBallPos.x, predictedBallPos.y);
		roboteam_utils::Vector2 getBallAtPos = roboteam_utils::Vector2(getBallAtX, getBallAtY);
		roboteam_utils::Vector2 ballTrajectory = ballPosThen - ballPosNow;
		roboteam_utils::Vector2 ballToCenter = getBallAtPos - ballPosNow;

		// ROS_INFO_STREAM("no robot has the ball, ");

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
			// ROS_INFO_STREAM("set targetAngle: " << (ballPosNow - getBallAtPos).angle());
			interceptAngle = (ballPosNow - getBallAtPos).angle();
		}
		interceptPose.interceptPos = interceptPos;
		interceptPose.interceptAngle = interceptAngle;
		
		return interceptPose;

	} else {
		// Predict intercept pos by looking at the robot that has the ball

		roboteam_msgs::WorldRobot otherRobot;
		if (hasBall < 6) {
			otherRobot = world.us.at(hasBall);
		} else {
			hasBall -= 6;
			otherRobot = world.them.at(hasBall);
		}

		// ROS_INFO_STREAM("robot " << hasBall << " has the ball!");

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
			double getBallAtX = GetDouble("getBallAtX");
			double getBallAtY = GetDouble("getBallAtY");
			double getBallAtTime = GetDouble("getBallAtTime");

			InterceptPose interceptPose = GetInterceptPos(getBallAtX, getBallAtY, getBallAtTime);
			roboteam_utils::Vector2 interceptPos = interceptPose.interceptPos;
			double interceptAngle = interceptPose.interceptAngle;
			if (interceptPos.x > 100.0 && interceptPos.y > 100.0) {
				// ROS_INFO("Ball is not estimated to be within range within the specified time, but I'll wait...");
				targetPos.x = getBallAtX;
				targetPos.y = getBallAtY;
				targetAngle = (ballPos-robotPos).angle();
			} else {
				if (interceptPos.x < -100.0 && interceptPos.y < -100.0) {
					ROS_INFO("Something went wrong in computing the intercept position :(");
					return Status::Invalid;
				}
				roboteam_utils::Vector2 getBallAtPos = roboteam_utils::Vector2(getBallAtX, getBallAtY);
				if (IsPointInCircle(getBallAtPos, acceptableDeviation, interceptPos)) {
					// ROS_INFO("Nice! This is probably a valid intercept position");
					targetPos = interceptPos;
					targetAngle = interceptAngle;
				} else {
					ROS_INFO("This is probably not a valid intercept position, please fix your code...");
					return Status::Invalid;
				}
			}
		} else {
			roboteam_utils::Vector2 posDiff = ballPos - robotPos;
			roboteam_utils::Vector2 posDiffNorm = posDiff.normalize();
			targetPos = ballPos - posDiffNorm.scale(0.09); // 0.09 = robot radius
			targetAngle = posDiff.angle();
			private_bb->SetBool("dribbler", true);
		}
	} else {
		roboteam_utils::Vector2 posDiff = ballPos - robotPos;
		roboteam_utils::Vector2 posDiffNorm = posDiff.normalize();
		targetPos = ballPos - posDiffNorm.scale(0.09); // 0.09 = robot radius
		targetAngle = posDiff.angle();
	}

	auto bb2 = std::make_shared<bt::Blackboard>();
	bb2->SetInt("me", robotID);
	bb2->SetBool("our_team",blackboard->GetBool("our_team"));
	IHaveBall iHaveBall("", bb2);

	if (iHaveBall.Update() == Status::Success) {
		roboteam_msgs::RobotCommand command;
		command.id = robotID;
		command.x_vel = 0.0;
		command.y_vel = 0.0;
		command.w = 0.0;
		command.dribbler = true;
		pubGetBall.publish(command);
		ros::spinOnce();
		ROS_INFO("GetBall skill completed.");
		return Status::Success;
	} else {
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        // ROS_INFO_STREAM("go to angle: " << targetAngle);
        private_bb->SetDouble("angleGoal", targetAngle);
        // private_bb->SetBool("endPoint", true);
        
        avoidRobots.Update();
		// goToPos.Update();
		prevTargetPos = roboteam_utils::Vector2(targetPos.x, targetPos.y);
		prevTargetAngle = targetAngle;
		return Status::Running;
	}
};

} // rtt
