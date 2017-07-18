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
    startTime = now();
}

void ReceiveBall::Initialize() {

	robotID = blackboard->GetInt("ROBOT_ID");	
	if (HasDouble("acceptableDeviation")) {
		acceptableDeviation = GetDouble("acceptableDeviation");
	} else {
		acceptableDeviation = 2.0;
	}

	ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", false);

	// Read the blackboard info about where to receive the ball 
	if ((HasDouble("receiveBallAtX") && HasDouble("receiveBallAtY"))) {
		double receiveBallAtX = GetDouble("receiveBallAtX");
		double receiveBallAtY = GetDouble("receiveBallAtY");
		receiveBallAtPos = Vector2(receiveBallAtX, receiveBallAtY);
	} else if (GetBool("computePoint")) {
		receiveBallAtPos = computePoint();
	} else {

		roboteam_msgs::WorldRobot robot;
		boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
	    if (findBot) {
	        robot = *findBot;
	        receiveBallAtPos = Vector2(robot.pos);
	    } else {
	        ROS_WARN("ReceiveBall could not find robot");
	    }
	}
}

Vector2 ReceiveBall::computePoint() {
	opportunityFinder.Initialize("spits.txt",robotID, "theirgoal", 0);
	if (HasDouble("computePointCloseToX") && HasDouble("computePointCloseToY")) {
		opportunityFinder.setCloseToPos(Vector2(GetDouble("computePointCloseToX"), GetDouble("computePointCloseToY")));
	}
	Vector2 receiveBallAtPos = opportunityFinder.computeBestOpportunity();
	prevComputedPoint = now();
	return receiveBallAtPos;
}

boost::optional<int> ReceiveBall::whichRobotHasBall() {
    auto holder = getBallHolder();
    if (!holder) {
        return boost::none;
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
	command.dribbler = false;
    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
}

// Predict intercept pos from ball velocity
InterceptPose ReceiveBall::deduceInterceptPosFromBall() {

	InterceptPose interceptPose;
	Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();
	// ballIsComing = false;

	bool avoidBall = GetBool("avoidBallsFromOurRobots") && our_team;

	Vector2 ballPos(world.ball.pos);
	Vector2 ballVel(world.ball.vel);
	double ballDir = ballVel.dot(receiveBallAtPos - ballPos);

	if (ballVel.length() < 0.1 || ballDir <= 0) {

		drawer.removeLine("ballTrajectory");
		drawer.removePoint("closestPointReceiveBall");
		interceptPose.interceptPos = receiveBallAtPos;
		interceptPose.interceptAngle = (ballPos - receiveBallAtPos).angle();
		
		return interceptPose;
	} else {
		Vector2 ballTrajectory = ballVel.scale(10.0 / ballVel.length());
		Vector2 closestPoint = ballTrajectory.closestPointOnVector(ballPos, receiveBallAtPos);

		drawer.setColor(255,255,255);
		drawer.drawLine("ballTrajectory", ballPos, ballTrajectory);
		drawer.drawPoint("closestPointReceiveBall", closestPoint);
		drawer.setColor(0,0,0);

		if ((closestPoint - receiveBallAtPos).length() < acceptableDeviation) {
			ballIsComing = true;
			interceptPos = closestPoint;
			interceptAngle = cleanAngle(ballVel.angle() + M_PI);

			if (avoidBall) {
				interceptPos = closestPoint + (receiveBallAtPos - closestPoint).normalize();
			}

		} else {
			interceptPos = receiveBallAtPos;
			interceptAngle = (ballPos - receiveBallAtPos).angle();
		}
		interceptPose.interceptPos = interceptPos;
		interceptPose.interceptAngle = interceptAngle;

		
		return interceptPose;
	}
}

// Predict intercept pos by looking at the robot that has the ball
boost::optional<InterceptPose> ReceiveBall::deduceInterceptPosFromRobot() {
	
	// ballIsComing = false;
	InterceptPose interceptPose;
	Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();

	roboteam_msgs::WorldRobot otherRobot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(*hasBall, our_team);
    if (findBot) {
        otherRobot = *findBot;
    } else {
        ROS_WARN("ReceiveBall could not find other robot");
        return interceptPose;
    }

	bool avoidBall = GetBool("avoidBallsFromOurRobots") && our_team;


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

		if (avoidBall) {
			interceptPos = closestPoint + (receiveBallAtPos - closestPoint).normalize();
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
	if ((HasDouble("receiveBallAtX") && HasDouble("receiveBallAtY"))) {
		double receiveBallAtX = GetDouble("receiveBallAtX");
		double receiveBallAtY = GetDouble("receiveBallAtY");
		receiveBallAtPos = Vector2(receiveBallAtX, receiveBallAtY);
	}

	
	// Wait for the first world message
	while (world.us.size() == 0) {
		ROS_INFO_STREAM("ReceiveBall, empty world...");
		return Status::Running;
	}

	// If we should use "opportunity finder" to compute a point to stand free, do it once per second
	if (HasBool("computePoint") && !ballIsComing) {
		if (time_difference_milliseconds(prevComputedPoint, now()).count() > 1000) {
			receiveBallAtPos = computePoint();
		}
	}

	// Check if the same robot still has the ball

	if (hasBall) {
	auto bb2 = std::make_shared<bt::Blackboard>();
		bb2->SetInt("me", *hasBall);
		bb2->SetBool("our_team", our_team);
		IHaveBall iHaveBall1("", bb2);
		if (iHaveBall1.Update() != Status::Success) {
			hasBall = whichRobotHasBall();
		}
	} else {
		hasBall = whichRobotHasBall();
	}
	// Store some info about the world state
	roboteam_msgs::WorldRobot robot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN("ReceiveBall could not find robot");
        return Status::Failure;
    }
	Vector2 robotPos(robot.pos);
	Vector2 ballPos(world.ball.pos);
	Vector2 ballVel(world.ball.vel);
	
	Vector2 targetPos;
	double targetAngle;
	bool ballWasComing = ballIsComing;
	ballIsComing = false;

	// Calculate where we can receive the ball close to the given receiveBallAt... point.
	boost::optional<InterceptPose> interceptPose;
	if (!hasBall || *hasBall == robotID) {
		interceptPose = deduceInterceptPosFromBall();
	} else {
		interceptPose = deduceInterceptPosFromRobot();
	}
	
	if (!interceptPose) {
		ROS_WARN("Receive ball was unable to calculate an InterceptPose");
		return Status::Failure;
	}

	Vector2 interceptPos = interceptPose->interceptPos;
	double interceptAngle = interceptPose->interceptAngle;



	// Determine if we should shoot at goal, depending whether the shootAtGoal boolean is set, and on whether we can see the goal
	double viewOfGoal = opportunityFinder.calcViewOfGoal(robotPos, world);
	bool canSeeGoal = viewOfGoal >= 0.2;
	double angleDiffBallGoal = fabs(cleanAngle((LastWorld::get_their_goal_center() - robotPos).angle() - (ballPos - robotPos).angle())); 
	bool shootAtGoal = GetBool("shootAtGoal") && canSeeGoal && angleDiffBallGoal <= 0.7*M_PI;


	// Set the targetAngle, if we should shoot at goal, we should face mostly towards the goal
	if (shootAtGoal) {
		drawer.setColor(255, 0, 0);
		double angleDiff = cleanAngle( ((ballPos - robotPos).angle() - (LastWorld::get_their_goal_center() - robotPos).angle()) );
		targetAngle = (LastWorld::get_their_goal_center() - robotPos).angle() + (angleDiff / 4.0);
		Vector2 robotRadius(0.095, 0.0);
		robotRadius = robotRadius.rotate(targetAngle);
		targetPos = interceptPos - robotRadius;
	} else {
		targetPos = interceptPos;
		targetAngle = interceptAngle;
	}


	// Set the readyToReceiveBall param once we're close
	Vector2 posError = receiveBallAtPos - robotPos;
	if (posError.length() < acceptableDeviation) {
		if (GetBool("setSignal")) {
			ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", true);
		}
		private_bb->SetBool("avoidRobots", false);
	} else {
		private_bb->SetBool("avoidRobots", true);
	}


	// double minRunTime = 2000;
	if (ballWasComing && !ballIsComing && GetBool("shouldFail") && posError.length() < acceptableDeviation) {
		ROS_INFO_STREAM("ROBOT " << robotID << " missed the ball");
		return Status::Failure;
	}

	if (ballVel.length() > 1.0 && !ballIsComing && GetBool("shouldFail") && posError.length() < acceptableDeviation) {
		ROS_INFO_STREAM("ball is probably not for us " << robotID);
		return Status::Failure;
	}


	// If we should shoot at the goal, we have to determine when the ball is going to reach us, so we can immediately shoot on
	double role_iterations_per_second = 0.0;
	ros::param::getCached("role_iterations_per_second", role_iterations_per_second);
	double timeStep;
	if (role_iterations_per_second == 0.0) {
		timeStep = 1.0 / 30.0;
	} else {
		timeStep = 1.0 / role_iterations_per_second;
	}

	double distanceToBall = (ballPos-robotPos).length();
	if (shootAtGoal) {
		if ((ballPos-receiveBallAtPos).length() < (ballVel.scale(timeStep).length() * 5.0)) {
			startKicking = true;
			kick.Initialize();
			return kick.Update();
		}
	}

	// if ((distanceToBall < acceptableDeviation && !ballIsComing && !(HasBool("dontDriveToBall") && GetBool("dontDriveToBall"))) || ballHasBeenClose) {
	// 	ballHasBeenClose = true;
	// 	getBallbb->SetInt("ROBOT_ID", robotID);
	// 	return getBall.Update();
	// }

	
	// If the ball gets close, turn on the dribbler
	double dribblerDist = acceptableDeviation * 2.0;
	if (shootAtGoal) {
		dribblerDist = 0.0;
	}

	if (distanceToBall < dribblerDist) {
		ROS_INFO_STREAM("dribbler on " << distanceToBall);
		private_bb->SetBool("dribbler", true);
	} else {
		ROS_INFO_STREAM("dribbler off " << distanceToBall);
		private_bb->SetBool("dribbler", false);
	}


	// Check the IHaveBall condition to see whether the GetBall skill succeeded
	auto bb3 = std::make_shared<bt::Blackboard>();
	bb3->SetInt("me", robotID);
	bb3->SetBool("our_team", true);
	IHaveBall iHaveBall2("", bb3);

	// double ballSpeed = Vector2(world.ball.vel.x, world.ball.vel.y).length();

    // if (iHaveBall2.Update() == Status::Success && ballSpeed < 0.1) {
    // if (iHaveBall2.Update() == Status::Success) {
	double angleError = cleanAngle(targetAngle - robot.angle);
	ROS_INFO_STREAM("receiveBall, dist " << distanceToBall << " angleError " << fabs(angleError));

	bool matchBallVel = false;
	if (distanceToBall <= 0.6 && fabs(angleError) <= 0.2) {
		matchBallVel = true;
	}

    if (distanceToBall <= 0.15 && fabs(angleError) <= 0.2) {
    	ROS_INFO("ReceiveBall success");
    	ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", false);
    	if (shootAtGoal) {
    		// ROS_INFO("Start kicking");
    		startKicking = true;
    		return kick.Update();
    	}
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
        private_bb->SetDouble("pGainPosition", 4.0);
        private_bb->SetBool("avoidDefenseAreas", true);
        if (HasString("stayOnSide")) {
        	private_bb->SetString("stayOnSide", GetString("stayOnSide"));
        }
        if (HasBool("stayAwayFromBall") && GetBool("stayAwayFromBall")) {
        	private_bb->SetBool("stayAwayFromBall", true);
        }

        boost::optional<roboteam_msgs::RobotCommand> commandPtr = goToPos.getVelCommand();
	    if (commandPtr) {
	        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();

	        roboteam_msgs::RobotCommand command = *commandPtr;
	        if (matchBallVel) {
		        ROS_INFO_STREAM("robot " << robotID << " matching ball vel");
		        Vector2 ballVelInRobotFrame = worldToRobotFrame(ballVel, robot.angle).scale(0.75);
		        Vector2 newVelCommand(command.x_vel + ballVelInRobotFrame.x, command.y_vel + ballVelInRobotFrame.y);
		        if (newVelCommand.length() > 4.0) {
		          newVelCommand.scale(4.0 / newVelCommand.length());
		        }
		        command.x_vel = newVelCommand.x;
		        command.y_vel = newVelCommand.y;    
		    }


	        pub.publish(command);
	    } else {
	    	roboteam_msgs::RobotCommand emptyCommand;
	    	auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	    	pub.publish(emptyCommand);
	    }

	    return Status::Running;		
	}
};

} // rtt
