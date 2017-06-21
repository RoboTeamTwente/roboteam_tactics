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
        , getBall("", blackboard)
        , kick("", blackboard) {
    hasBall = whichRobotHasBall();
    
    prevComputedPoint = now();
    computedTargetPos = false;
}

void ReceiveBall::Initialize() {

	robotID = blackboard->GetInt("ROBOT_ID");	
	if (HasDouble("acceptableDeviation")) {
		acceptableDeviation = GetDouble("acceptableDeviation");
	} else {
		acceptableDeviation = 1.0;
	}

	

	// Read the blackboard info about where to receive the ball 
	if ((HasDouble("receiveBallAtX") && HasDouble("receiveBallAtY"))) {
		double receiveBallAtX = GetDouble("receiveBallAtX");
		double receiveBallAtY = GetDouble("receiveBallAtY");
		receiveBallAtPos = Vector2(receiveBallAtX, receiveBallAtY);
	} else if (GetBool("computePoint")) {
		receiveBallAtPos = computePoint();
	} else {
		roboteam_msgs::WorldRobot robot = *getWorldBot(robotID);
		receiveBallAtPos = Vector2(robot.pos);
	}
}

Vector2 ReceiveBall::computePoint() {
	passPoint.Initialize("spits.txt",robotID, "theirgoal", 0);
	if (HasDouble("computePointCloseToX") && HasDouble("computePointCloseToY")) {
		passPoint.setCloseToPos(Vector2(GetDouble("computePointCloseToX"), GetDouble("computePointCloseToY")));
	}
	Vector2 receiveBallAtPos = passPoint.computeBestPassPoint();
	prevComputedPoint = now();
	return receiveBallAtPos;
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
InterceptPose ReceiveBall::deduceInterceptPosFromBall() {

	InterceptPose interceptPose;
	Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();

	Vector2 ballPos(world.ball.pos);
	Vector2 ballVel(world.ball.vel);
	double ballDir = ballVel.dot(receiveBallAtPos - ballPos);
	if (ballVel.length() < 0.1 || ballDir <= 0) {
		interceptPose.interceptPos = receiveBallAtPos;
		interceptPose.interceptAngle = (ballPos - receiveBallAtPos).angle();
		enableComputePoint = true;
		return interceptPose;
	} else {
		Vector2 ballTrajectory = ballVel.scale(5.0 / ballVel.length());
		Vector2 closestPoint = ballTrajectory.closestPointOnVector(ballPos, receiveBallAtPos);

		if ((closestPoint - receiveBallAtPos).length() < acceptableDeviation) {
			interceptPos = closestPoint;
			interceptAngle = cleanAngle(ballVel.angle() + M_PI);
		} else {
			interceptPos = receiveBallAtPos;
			interceptAngle = cleanAngle(ballVel.angle() + M_PI);
		}
		interceptPose.interceptPos = interceptPos;
		interceptPose.interceptAngle = interceptAngle;

		enableComputePoint = false;
		return interceptPose;
	}
}

// Predict intercept pos by looking at the robot that has the ball
InterceptPose ReceiveBall::deduceInterceptPosFromRobot() {
	
	enableComputePoint = true;
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

	// If the robot with the ball is facing away from us, we should not rotate to receive the ball from him
	double ballDir = otherRobotLooksAt.dot(receiveBallAtPos - ballPosNow);
	if (ballDir <= 0) {
		interceptPose.interceptPos = receiveBallAtPos;
		interceptPose.interceptAngle = (ballPosNow - receiveBallAtPos).angle();
		return interceptPose;
	}

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

bt::Node::Status ReceiveBall::Update() {
	
	if (startKicking) {
		return kick.Update();
	}

    // Get the last world information and some blackboard info
	roboteam_msgs::World world = LastWorld::get();
	// robotID = blackboard->GetInt("ROBOT_ID");
	if ((HasDouble("receiveBallAtX") && HasDouble("receiveBallAtY"))) {
		double receiveBallAtX = GetDouble("receiveBallAtX");
		double receiveBallAtY = GetDouble("receiveBallAtY");
		receiveBallAtPos = Vector2(receiveBallAtX, receiveBallAtY);
	}

	

	// Wait for the first world message
	while (world.us.size() == 0) {
		return Status::Running;
	}

	if (HasBool("computePoint") && enableComputePoint) {
		if (time_difference_milliseconds(prevComputedPoint, now()).count() > 1000) {
			receiveBallAtPos = computePoint();
			// ROS_INFO_STREAM("Robot " << robotID << " computed point: " << receiveBallAtPos);
		}
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
	Vector2 ballVel(world.ball.vel);
	
	Vector2 targetPos;
	double targetAngle;
	

	// Calculate where we can receive the ball close to the given receiveBallAt... point.
	InterceptPose interceptPose;
	if (hasBall == -1 || hasBall == robotID) {
		interceptPose = deduceInterceptPosFromBall();
	} else {
		interceptPose = deduceInterceptPosFromRobot();
	}


	Vector2 interceptPos = interceptPose.interceptPos;
	double interceptAngle = interceptPose.interceptAngle;

	double viewOfGoal = passPoint.calcViewOfGoal(robotPos, world);
	// ROS_INFO_STREAM("robot " << robotID << " receiveBall viewOfGoal: " << viewOfGoal);
	bool canSeeGoal = viewOfGoal >= 0.2;
	bool shootAtGoal = GetBool("shootAtGoal") && canSeeGoal;
	// ROS_INFO_STREAM("receiveBall shootAtGoal: " << shootAtGoal);

	if (shootAtGoal) {
		drawer.setColor(255, 0, 0);
		double angleDiff = cleanAngle( ((ballPos - robotPos).angle() - (LastWorld::get_their_goal_center() - robotPos).angle()) );
		targetAngle = (LastWorld::get_their_goal_center() - robotPos).angle() + (angleDiff / 4.0);
		// targetAngle = (LastWorld::get_their_goal_center() - robotPos).angle();

		Vector2 robotRadius(0.09, 0.0);
		robotRadius = robotRadius.rotate(targetAngle);
		targetPos = interceptPos - robotRadius;
	} else {
		targetPos = interceptPos;
		targetAngle = interceptAngle;
	}


	Vector2 posError = receiveBallAtPos - robotPos;
	if (posError.length() < acceptableDeviation) {
		ROS_INFO_STREAM("robot " << robotID << " readyToReceiveBall");
		ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", true);
		private_bb->SetBool("avoidRobots", false);
	} else {
		private_bb->SetBool("avoidRobots", true);
	}


	// If the ball is within reach and lying still or moving slowy, we should drive towards it
	double role_iterations_per_second = 0.0;
	ros::param::getCached("role_iterations_per_second", role_iterations_per_second);
	double timeStep;
	if (role_iterations_per_second == 0.0) {
		timeStep = 1.0 / 30.0;
	} else {
		timeStep = 1.0 / role_iterations_per_second;
	}
	

	double distanceToBall = (ballPos-interceptPos).length();
	if (shootAtGoal) {
		if ((ballPos-receiveBallAtPos).length() < (ballVel.scale(timeStep).length() * 5.0)) {
			startKicking = true;
			kick.Initialize();
			return kick.Update();
		}
	}

	if ((distanceToBall < acceptableDeviation && ballVel.length() < 0.5)|| ballHasBeenClose) {
		// ballHasBeenClose = true;
		return getBall.Update();
	}

	
	// If the ball gets close, turn on the dribbler
	double dribblerDist = acceptableDeviation * 2.0;
	if (shootAtGoal) {
		dribblerDist = 0.0;
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

    if (iHaveBall2.Update() == Status::Success && ballSpeed < 0.1) {
    	ROS_INFO("ReceiveBall success");
    	if (shootAtGoal) {
    		ROS_INFO("Start kicking");
    		startKicking = true;
    		return kick.Update();
    	}
		RTT_DEBUGLN("ReceiveBall skill completed.");
		publishStopCommand();
		ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", false);
		return Status::Success;
	} else {
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        private_bb->SetBool("avoidDefenseAreas", true);

        boost::optional<roboteam_msgs::RobotCommand> command = goToPos.getVelCommand();
	    if (command) {
	        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	        pub.publish(*command);
	    }

	    return Status::Running;		
	}
};

} // rtt