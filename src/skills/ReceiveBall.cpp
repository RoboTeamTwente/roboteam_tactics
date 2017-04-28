#include "ros/ros.h"

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/ReceiveBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"

#include <cmath>
#include <vector>
#include <string>

#define RTT_CURRENT_DEBUG_TAG ReceiveBall

namespace rtt {

RTT_REGISTER_SKILL(ReceiveBall);

ReceiveBall::ReceiveBall(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb)
        , getBall("", blackboard) {
    hasBall = whichRobotHasBall();
    ros::param::set("readyToReceiveBall", false);
}

int ReceiveBall::whichRobotHasBall() {
    auto holder = getBallHolder();
    if (!holder) {
        return -1;
    }
    our_team = holder->second;
    return holder->first.id;
}

void ReceiveBall::publishStopCommand() {
	roboteam_msgs::RobotCommand command;
	command.id = robotID;
	command.x_vel = 0.0;
	command.y_vel = 0.0;
	command.w = 0.0;
	command.dribbler = true;
    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
}

// Predict intercept pos from ball velocity
InterceptPose ReceiveBall::deduceInterceptPosFromBall(double receiveBallAtX, double receiveBallAtY) {

	InterceptPose interceptPose;
	Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();

	double timeToLookIntoFuture = 5.0; // in seconds
	Vector2 ballPosNow(world.ball.pos.x, world.ball.pos.y);
	Vector2 ballPosThen(LastWorld::predictBallPos(timeToLookIntoFuture));
	Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
	Vector2 ballTrajectory = ballPosThen - ballPosNow;
	Vector2 ballToCenter = receiveBallAtPos - ballPosNow;

	drawer.drawLine("ballTrajectory", ballPosNow, ballTrajectory);

	double ballTrajectoryMagn = ballTrajectory.length();
	ballTrajectory = ballTrajectory.scale(1/ballTrajectoryMagn);
	double projectionOnBallTrajectory = ballToCenter.dot(ballTrajectory);
	Vector2 closestPoint = ballPosNow + ballTrajectory*projectionOnBallTrajectory;

	// If the ball is moving (towards us) and the computed closest point is within range, then stand on the closest point
	bool ballMovingTowardsUs = (ballPosThen-ballPosNow).dot(receiveBallAtPos-ballPosNow) >= 0;
	if (ballMovingTowardsUs && ballTrajectoryMagn > 0.1 && (closestPoint - receiveBallAtPos).length() < acceptableDeviation) {
		ROS_INFO_STREAM("deduce from ballvel, pos: " << interceptPos.x << " " << interceptPos.y);
		interceptPos = closestPoint;
		interceptAngle = (ballPosNow-ballPosThen).angle();
	} else {
		interceptPos = receiveBallAtPos;
		interceptAngle = (ballPosNow - receiveBallAtPos).angle();
	}
	interceptPose.interceptPos = interceptPos;
	interceptPose.interceptAngle = interceptAngle;


	
	return interceptPose;
}

// Predict intercept pos by looking at the robot that has the ball
InterceptPose ReceiveBall::deduceInterceptPosFromRobot(double receiveBallAtX, double receiveBallAtY) {
	
	InterceptPose interceptPose;
	Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();

	roboteam_msgs::WorldRobot otherRobot = *getWorldBot(hasBall, our_team);

	// If the other robot would shoot now, use its orientation to estimate the ball trajectory, and then the closest
	// point on this trajectory to our robot, so he can receive the ball there
	Vector2 otherRobotLooksAt = Vector2(1, 0);
	otherRobotLooksAt = otherRobotLooksAt.rotate(otherRobot.angle);
	Vector2 ballPosNow(world.ball.pos.x, world.ball.pos.y);
	Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
	Vector2 ballToCenter = receiveBallAtPos - ballPosNow;
	double projectionOnBallTrajectory = ballToCenter.dot(otherRobotLooksAt);
	Vector2 closestPoint = ballPosNow + otherRobotLooksAt*projectionOnBallTrajectory;

	// If the computed closest point is within range, go stand there. Otherwise wait at the specified receiveBallAt... position
	if ((closestPoint - receiveBallAtPos).length() < acceptableDeviation) {
		interceptPos = closestPoint;
		if (otherRobot.angle > 0) {
			interceptAngle = otherRobot.angle - M_PI;
		} else {
			interceptAngle = otherRobot.angle + M_PI;
		}
	} else {
		interceptPos = receiveBallAtPos;
		interceptAngle = (ballPosNow - receiveBallAtPos).angle();
	}

	interceptPose.interceptPos = interceptPos; 
	interceptPose.interceptAngle = interceptAngle;
	return interceptPose;
}

bt::Node::Status ReceiveBall::Update (){

    // Get the last world information and some blackboard info
	roboteam_msgs::World world = LastWorld::get();
	robotID = blackboard->GetInt("ROBOT_ID");
	if (HasDouble("acceptableDeviation")) {
		acceptableDeviation = GetDouble("acceptableDeviation");
	} else {
		acceptableDeviation = 1.0;
	}


	// Wait for the first world message
	while (world.us.size() == 0) {
		return Status::Running;
	}


	// Check if the same robot still has the ball
	auto bb2 = std::make_shared<bt::Blackboard>();
	bb2->SetInt("me", hasBall);
	bb2->SetBool("our_team", our_team);
	IHaveBall iHaveBall1("", bb2);
	if (iHaveBall1.Update() != Status::Success) {
		hasBall = whichRobotHasBall();
	}


	// Store some info about the world state
	roboteam_msgs::WorldBall ball = world.ball;
	roboteam_msgs::WorldRobot robot = *getWorldBot(robotID);
	Vector2 ballPos = Vector2(ball.pos.x, ball.pos.y);
	Vector2 robotPos = Vector2(robot.pos.x, robot.pos.y);
	double robotAngle = robot.angle;
	
	Vector2 targetPos;
	double targetAngle;


	// Read the blackboard info about where to receive the ball 
	double receiveBallAtX;
	double receiveBallAtY;
	if (HasBool("receiveBallAtCurrentPos") || (HasDouble("receiveBallAtX") && HasDouble("receiveBallAtY"))) {
		if (GetBool("receiveBallAtCurrentPos")) {
			receiveBallAtX = robotPos.x;
			receiveBallAtY = robotPos.y;
		} else {
			receiveBallAtX = GetDouble("receiveBallAtX");
			receiveBallAtY = GetDouble("receiveBallAtY");
		}
	} else {
		ROS_WARN("ReceiveBall blackboard is probably not set well, I'll just asssume I have to receive the ball at my current position");
		receiveBallAtX = robotPos.x;
		receiveBallAtY = robotPos.y;
	}


	// Calculate where we can receive the ball close to the given receiveBallAt... point.
	InterceptPose interceptPose;
	if (hasBall == -1) {
		interceptPose = deduceInterceptPosFromBall(receiveBallAtX, receiveBallAtY);
	} else {
		interceptPose = deduceInterceptPosFromRobot(receiveBallAtX, receiveBallAtY);
		// interceptPose = deduceInterceptPosFromBall(receiveBallAtX, receiveBallAtY);
	}
	Vector2 interceptPos = interceptPose.interceptPos;
	double interceptAngle = interceptPose.interceptAngle;


	// Check whether the point returned by the deduceInterceptPos... function actually returns a point that is close (less than acceptableDeviation)
	// from the specified receiveBallAt... point. This should always be the case (maybe move this check to some test function rather than perform
	// it in the skill update)
	Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
	if (isPointInCircle(receiveBallAtPos, acceptableDeviation, interceptPos)) {
		targetPos = interceptPos;
		targetAngle = interceptAngle;
	} else {
		ROS_ERROR("This is probably not a valid intercept position...");
		return Status::Failure;
	}


	// If we are too far from the ball, or too far from the speficied targetPos, we should drive towards the targetPos
	Vector2 posError = targetPos - robotPos;
	double distanceToBall = (ballPos-receiveBallAtPos).length();
	double acceptableDeviation2 = 1.5;

	Vector2 ballVel(ball.vel);
	ROS_INFO_STREAM("ballVel: " << ballVel.length());
	if (ballHasBeenClose || (distanceToBall < acceptableDeviation2 && ballVel.length() < 0.5)) {
		ballHasBeenClose = true;
		return getBall.Update();
	}

	// if (distanceToBall > acceptableDeviation2 || posError.length() > acceptableDeviation2) {
	// 	double angleError = targetAngle - robotAngle;
	// 	if (posError.length() <= 0.1 && fabs(angleError) < 0.3) {
	// 		// Set a rosparam to let other robots know that we are ready to receive the ball
	// 		ros::param::set("readyToReceiveBall", true);
	// 	}
	// 	// private_bb->SetBool("dribbler", false);
	// } else { 
	// 	// If we are close enough to the ball we can drive towards it and turn on the dribbler
	// 	return getBall.Update();

	// 	Vector2 posDiff = ballPos - robotPos;
	// 	Vector2 posDiffNorm = posDiff.normalize();
	// 	targetPos = ballPos - posDiffNorm.scale(0.09); // 0.09 = robot radius
	// 	targetAngle = posDiff.angle();
	// 	// private_bb->SetBool("dribbler", true);
	// }
	
	if (distanceToBall < 2.0) {
		private_bb->SetBool("dribbler", true);
	} else {
		private_bb->SetBool("dribbler", false);
	}

	// Check the IHaveBall condition to see whether the GetBall skill succeeded
	auto bb3 = std::make_shared<bt::Blackboard>();
	bb3->SetInt("me", robotID);
	bb3->SetBool("our_team", true);
	IHaveBall iHaveBall2("", bb3);

	double ballSpeed = Vector2(world.ball.vel.x, world.ball.vel.y).length();

    if (iHaveBall2.Update() == Status::Success && ballSpeed < 0.1) {
		RTT_DEBUGLN("ReceiveBall skill completed.");
		publishStopCommand();
		// return Status::Running;
		return Status::Success;
	} else {
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        private_bb->SetBool("avoidRobots", true);
        if (HasDouble("pGainPosition")) {
        	private_bb->SetDouble("pGainPosition", GetDouble("pGainPosition"));
        }
        if (HasDouble("successDist")) {
        	private_bb->SetDouble("successDist", GetDouble("successDist"));
        }
        goToPos.getVelCommand();

        // goToPos.Update();
        boost::optional<roboteam_msgs::RobotCommand> command = goToPos.getVelCommand();
	    if (command) {
	        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	        pub.publish(*command);
	    }

	    return Status::Running;		
	}
};

} // rtt
