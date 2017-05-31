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
		, paramName(name + "_readyToReceiveBall")
        , goToPos("", private_bb)
        , getBall("", blackboard) {
    hasBall = whichRobotHasBall();
    ros::param::set(paramName, false);
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
Position ReceiveBall::deduceInterceptPosFromBall(double receiveBallAtX, double receiveBallAtY) {

	Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();

	double timeToLookIntoFuture = 5.0; // in seconds
	Vector2 ballPosNow(world.ball.pos.x, world.ball.pos.y);
	Vector2 ballPosThen(LastWorld::predictBallPos(timeToLookIntoFuture));
	// Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
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

	return {interceptPos.x, interceptPos.y, interceptAngle};
}

// Predict intercept pos by looking at the robot that has the ball
Position ReceiveBall::deduceInterceptPosFromRobot(double receiveBallAtX, double receiveBallAtY) {
	
	Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();

	roboteam_msgs::WorldRobot otherRobot = *getWorldBot(hasBall, our_team);

	// If the other robot would shoot now, use its orientation to estimate the ball trajectory, and then the closest
	// point on this trajectory to our robot, so he can receive the ball there
	Vector2 otherRobotLooksAt = Vector2(1, 0);
	otherRobotLooksAt = otherRobotLooksAt.rotate(otherRobot.angle);
	Vector2 ballPosNow(world.ball.pos.x, world.ball.pos.y);
	// Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
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

	return {interceptPos.x, interceptPos.y, interceptAngle};
}

void ReceiveBall::setParam(Position targetPos) {
	auto bot = *getWorldBot(robotID);
	Position pos = {bot.pos.x, bot.pos.y, bot.angle};
	// Must be within 1 cm (=sqrt(.0001) m) and .05 rad of the target position
	if (pos.location().dist2(targetPos.location()) < .0001 && fabs(pos.rot - targetPos.rot) < .05) {
		ros::param::set(paramName, true);
		ros::param::set("ready_to_pass", true);
		RTT_DEBUGLN("Ready! Setting params!");
	} else {
		RTT_DEBUGLN("Not ready yet...");
	}
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
	//double robotAngle = robot.angle;
	
	Position targetPos;

	// Read the blackboard info about where to receive the ball 
	double receiveBallAtX;
	double receiveBallAtY;
	if (HasBool("receiveBallAtCurrentPos") || (HasDouble("receiveBallAtX") && HasDouble("receiveBallAtY"))) {
		if (GetBool("receiveBallAtCurrentPos")) {
			receiveBallAtX = robotPos.x;
			receiveBallAtY = robotPos.y;
			receiveBallAtPos = robotPos;
		} else {
			receiveBallAtX = GetDouble("receiveBallAtX");
			receiveBallAtY = GetDouble("receiveBallAtY");
			receiveBallAtPos = Vector2(receiveBallAtX, receiveBallAtY);
		}
	} else if (HasBool("computePoint") && GetBool("computePoint")) {
		if (time_difference_milliseconds(prevCheck, now()).count() >= 1000) {
			passPoint.Initialize("spits.txt");
			receiveBallAtPos = passPoint.computeBestPassPoint(robotID, "theirgoal", 0);
			ROS_INFO_STREAM("ReceiveBall computed point: " << receiveBallAtPos);
			receiveBallAtX = receiveBallAtPos.x;
			receiveBallAtY = receiveBallAtPos.y;
			prevCheck = now();
		} else {
			ROS_INFO_STREAM("not chekcing");
		}
	} else {
		ROS_WARN("ReceiveBall blackboard is probably not set well, I'll just assume I have to receive the ball at my current position");
		receiveBallAtX = robotPos.x;
		receiveBallAtY = robotPos.y;
		receiveBallAtPos = robotPos;
	}


	// Calculate where we can receive the ball close to the given receiveBallAt... point.
	Position interceptPos;
	if (hasBall == -1) {
		interceptPos = deduceInterceptPosFromBall(receiveBallAtX, receiveBallAtY);
	} else {
		interceptPos = deduceInterceptPosFromRobot(receiveBallAtX, receiveBallAtY);
	}


	// Check whether the point returned by the deduceInterceptPos... function actually returns a point that is close (less than acceptableDeviation)
	// from the specified receiveBallAt... point. This should always be the case (maybe move this check to some test function rather than perform
	// it in the skill update)
	Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
	if (isPointInCircle(receiveBallAtPos, acceptableDeviation, interceptPos.location())) {
		targetPos = interceptPos;
	} else {
		ROS_ERROR("This is probably not a valid intercept position... @(%f, %f) ->(%f, %f)",
				receiveBallAtPos.x, receiveBallAtPos.y, interceptPos.x, interceptPos.y);
		return Status::Failure;
	}


	// If the ball is within reach and lying still or moving slowy, we should drive towards it
	double distanceToBall = (ballPos-receiveBallAtPos).length();
	Vector2 ballVel(ball.vel);
	if (ballHasBeenClose || (distanceToBall < acceptableDeviation && ballVel.length() < 0.5)) {
		ballHasBeenClose = true;
		return getBall.Update();
	}
	
	// If the ball gets close, turn on the dribbler
	double dribblerDist = acceptableDeviation;
	if (HasDouble("dribblerDist")) {
		dribblerDist = GetDouble("dribblerDist");
	}

	if (distanceToBall < dribblerDist) {
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

	setParam(targetPos);

    if (iHaveBall2.Update() == Status::Success && ballSpeed < 0.1) {
		RTT_DEBUGLN("ReceiveBall skill completed.");
		publishStopCommand();
		return Status::Success;
	} else {
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetPos.rot);
        private_bb->SetBool("avoidRobots", true);

        // if (HasDouble("pGainPosition")) {
        // 	private_bb->SetDouble("pGainPosition", GetDouble("pGainPosition"));
        // }
        // if (HasDouble("successDist")) {
        // 	private_bb->SetDouble("successDist", GetDouble("successDist"));
        // }

        boost::optional<roboteam_msgs::RobotCommand> command = goToPos.getVelCommand();
	    if (command) {
	        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	        pub.publish(*command);
	    }

	    return Status::Running;		
	}
};

} // rtt
