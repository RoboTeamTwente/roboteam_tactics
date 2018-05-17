#include "ros/ros.h"

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/ReceiveBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/conditions/IsInDefenseArea.h"

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
        // , kick("", blackboard)
        // , isRobotClosestToBall("", blackboard)
        {
    hasBall = whichRobotHasBall();

    prevComputedPoint = now();
    //computedTargetPos = false;

}

void ReceiveBall::Initialize() {
	hasTerminated = false;
	startTime = now();
	robotID = blackboard->GetInt("ROBOT_ID");
	ROS_INFO_STREAM_NAMED("skills.ReceiveBall", "Initialize for robot: " << robotID);
	ballIsComing = false;
	startKicking = false;

	// readyHasBeenSet = false;
	//	If we set readyToReceiveBall true, my teammate that passes the ball can give me the claim over the ball
	//	(such that other robots wont interfere). If setSignal is enabled, I can be trusted in resetting claim if this skill terminates.
	if (GetBool("setSignal")) {
		ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", true);
	} else {
		ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", false);
	}


	if (HasDouble("acceptableDeviation")) {
		acceptableDeviation = GetDouble("acceptableDeviation");
	} else {
		acceptableDeviation = 2.0;
	}
	if (HasDouble("marginDeviation")) {
		marginDeviation = GetDouble("marginDeviation");
	} else {
		marginDeviation = 0.0;
	}



	// Read the blackboard info about where to receive the ball
	if ((HasDouble("receiveBallAtX") && HasDouble("receiveBallAtY"))) {
		double receiveBallAtX = GetDouble("receiveBallAtX");
		double receiveBallAtY = GetDouble("receiveBallAtY");
		receiveBallAtPos = Vector2(receiveBallAtX, receiveBallAtY);
	} else if (GetBool("computePoint")) {
		receiveBallAtPos = computePoint();
	} else if (GetBool("claimedPos")) {
		double botClaimedX;
		double botClaimedY;
		ros::param::getCached("robot" + std::to_string(robotID) + "/claimedPosX", botClaimedX);
		ros::param::getCached("robot" + std::to_string(robotID) + "/claimedPosY", botClaimedY);
		receiveBallAtPos = Vector2(botClaimedX, botClaimedY);
		ROS_DEBUG_STREAM_NAMED("skills.ReceiveBall", "robot " << robotID << ", receiving ball at claimed pos: x: " << botClaimedX << ", y: " << botClaimedY);
	} else {

		roboteam_msgs::WorldRobot robot;
		boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
	    if (findBot) {
	        robot = *findBot;
	        receiveBallAtPos = Vector2(robot.pos);
	    } else {
	        ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "ReceiveBall could not find robot");
	    }
	}
}

void ReceiveBall::Terminate(bt::Node::Status s) {

	if (!hasTerminated) { // Temporary hack, because terminate is not always called at the right moments
		hasTerminated = true;

		if (GetBool("setSignal")) {
		// Check if the robot actually expects to receive a pass
		// If setSignal is not used, the rosparam robotClaimedBall will not be set to my ID (it shouldnt, at least)

			int robotClaimedBall;
		    ros::param::get("robotClaimedBall", robotClaimedBall);
		    if (robotClaimedBall == robotID) {
		    	ros::param::set("robotClaimedBall", -1);
        		ROS_DEBUG_STREAM_NAMED("skills.ReceiveBall", robotID << " released ball");
		    }

			ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", false); // reset readyToReceiveBall
			int passToParam;
		    ros::param::get("passToRobot", passToParam);
		    if (passToParam == robotID) {
		        ros::param::set("passToRobot", -1);
		        ROS_INFO_STREAM_NAMED("skills.ReceiveBall", "Terminate for " << robotID << ", resetting passToRobot and readyToReceiveBall");
		    } else {
		    	ROS_INFO_STREAM_NAMED("skills.ReceiveBall", "Terminate for " << robotID << ", resetting readyToReceiveBall");
		    }

		}
	}

}

Vector2 ReceiveBall::computePoint() {
	// opportunityFinder.Initialize("spits.txt",robotID, "theirgoal", 0);
	// if (HasDouble("computePointCloseToX") && HasDouble("computePointCloseToY")) {
	// 	opportunityFinder.setCloseToPos(Vector2(GetDouble("computePointCloseToX"), GetDouble("computePointCloseToY")));
	// }

	receiveBallAtPos = Vector2(GetDouble("computePointCloseToX"), GetDouble("computePointCloseToY"));

	// Vector2 receiveBallAtPos = opportunityFinder.computeBestOpportunity();
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

	if (startKicking) {
		command.kicker = true;
		command.kicker_vel = 6.5;
	}

    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
}

// Predict intercept pos from ball velocity
InterceptPose ReceiveBall::deduceInterceptPosFromBall(Vector2 ballPos, Vector2 ballVel, Vector2 myPos) {

	InterceptPose interceptPose;
	Vector2 interceptPos;
	double interceptAngle;

	bool avoidBall = GetBool("avoidBallsFromOurRobots") && our_team;

	// Is ball coming towards me? (for determining failure/success)
	Vector2 posdiff = myPos - ballPos;
	double velThreshold = posdiff.length()/4; // ARBITRARY GUESS
	if (GetBool("defenderMode")) {
		velThreshold = 0.1;
	} else if (velThreshold < 0.1) {
		velThreshold = 0.1;
	}
	if (ballVel.dot(posdiff.stretchToLength(1)) < velThreshold) { // if the velocity in my direction is too low
		if(!ballIsComing && !GetBool("defenderMode") && GetBool("setSignal")) {
			// if a pass is noted to myself, I assume the ball is coming.
			int robotClaimedBall;
		    ros::param::getCached("robotClaimedBall", robotClaimedBall);
		    if (robotClaimedBall == robotID) {
		    	ballIsComing = true;
	    		ROS_DEBUG_STREAM_NAMED("skills.ReceiveBall", robotID << " noted a pass to himself, and no teammate has ball, so assumes ball is coming");
		    }
		} else {
			ballIsComing = false;
		}
	}//--------------------------

	// Actual determining of the interceptPose
	if (ballVel.length() < 0.1) {
		interceptPose.interceptPos = receiveBallAtPos;
		interceptPose.interceptAngle = (ballPos - receiveBallAtPos).angle();
	} else {
		// Vector2 ballTrajectory = ballVel.stretchToLength(10.0);
		Vector2 closestPoint = ballPos + (receiveBallAtPos - ballPos).project2(ballVel); //ballTrajectory.closestPointOnVector(ballPos, receiveBallAtPos);
		ballIsComing = true;

		if ((closestPoint - receiveBallAtPos).length() < acceptableDeviation) {
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
	}

	return interceptPose;
}

// Predict intercept pos by looking at the robot that has the ball
boost::optional<InterceptPose> ReceiveBall::deduceInterceptPosFromRobot() {

	InterceptPose interceptPose;
	Vector2 interceptPos;
	roboteam_msgs::World world = LastWorld::get();

	roboteam_msgs::WorldRobot otherRobot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(*hasBall, our_team);
    if (findBot) {
        otherRobot = *findBot;
    } else {
        ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "ReceiveBall could not find other robot");
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
	double deviation = (closestPoint - receiveBallAtPos).length();
	// If the computed closest point is within range, go stand there. Otherwise wait at the specified receiveBallAt... position
	if (deviation > 0.001 && deviation < acceptableDeviation + marginDeviation) {
		// if the deviation is within the margin region, it should remain at acceptableDeviation until it exceeds the margin
		if (deviation > acceptableDeviation) {
			deviation = acceptableDeviation;
		}
		if (avoidBall) {
			deviation = deviation - 1;
		}
		interceptPos = receiveBallAtPos + (closestPoint - receiveBallAtPos).stretchToLength(deviation);
	} else {
		interceptPos = receiveBallAtPos;
	}

	interceptPose.interceptPos = interceptPos;
	interceptPose.interceptAngle = (ballPosNow - interceptPos).angle();
	return interceptPose;
}

bt::Node::Status ReceiveBall::Update() {
	if (hasTerminated) { // Temporary hack, because terminate is not always called at the right moments (similar hack in terminate function)
        Initialize();
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
        ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "ReceiveBall could not find robot");
        return Status::Failure;
    }
	Vector2 myPos(robot.pos);
	Vector2 ballPos(world.ball.pos);
	Vector2 ballVel(world.ball.vel);

	Vector2 targetPos;
	double targetAngle;
	bool ballWasComing = ballIsComing;

	// Calculate where we can receive the ball close to the given receiveBallAt... point.
	boost::optional<InterceptPose> interceptPose;
	if (GetBool("defenderMode") && (!hasBall || *hasBall == robotID)) { 			// if im defending and no other robot has the ball
		interceptPose = deduceInterceptPosFromBall(ballPos, ballVel, myPos);			// use ball for determining interceptpos
		// ballIsComing is also determined in this function
	} else if (!GetBool("defenderMode") && (!hasBall || (hasBall && !our_team))) { 	// if im not defending and no teammate has the ball
		interceptPose = deduceInterceptPosFromBall(ballPos, ballVel, myPos);			// use ball for determining interceptpos
		// ballIsComing is also determined in this function
	} else {																		// else
		interceptPose = deduceInterceptPosFromRobot();									// use robot in possession for determining interceptpos
		ballIsComing = false;
	}

	if (!interceptPose) {
		ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "Receive ball was unable to calculate an InterceptPose");
		return Status::Failure;
	}
	Vector2 interceptPos = interceptPose->interceptPos;
	double interceptAngle = interceptPose->interceptAngle;

	// isRobotClosestToBall.Update() == Status::Success
    	// 	int elapsedTime = time_difference_milliseconds(startTime, now()).count();
    	// 	if (elapsedTime > 200 && ) {
    	// 		ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "robot " << robotID << " failed because im closest to a slow moving ball");
    	// 		return Status::Failure;
    	// 	}
    	// } else {
    	// 	startTime = now();


	// Determine if we should shoot at goal, depending whether the shootAtGoal boolean is set, and on whether we can see the goal
	// and on whether the 'reflection angle' (angleDiff) is small enough
	bool shootAtGoal = false;
	double angleDiff = 0;
	if (GetBool("shootAtGoal")) {
		double viewOfGoal = opportunityFinder.calcViewOfGoal(receiveBallAtPos, world); // chosen reception pos is used to assess view of goal
		angleDiff = cleanAngle( ((ballPos - myPos).angle() - (LastWorld::get_their_goal_center() - myPos).angle()) );
		shootAtGoal = (viewOfGoal > 0.1 && fabs(angleDiff) < M_PI/2);
	}


	// Set the targetAngle, if we should shoot at goal, we should face mostly towards the goal
	if (shootAtGoal) {
		drawer.setColor(255, 0, 0);
		targetAngle = (LastWorld::get_their_goal_center() - myPos).angle() + (angleDiff / 4.0);
		Vector2 robotRadius(0.095, 0.0);
		robotRadius = robotRadius.rotate(targetAngle);
		targetPos = interceptPos - robotRadius;
	} else {
		targetPos = interceptPos;
		targetAngle = interceptAngle;
	}


	// // Once we're close, turn off robot avoidance, and set the readyToReceiveBall param if necessary
	// Vector2 posError = targetPos - myPos; //receiveBallAtPos - myPos;
	// double readyDist = 0.5;
	// if (HasDouble("readyDist")) {
	// 	readyDist = GetDouble("readyDist");
	// }
	// if (posError.length() < readyDist) {
	// 	private_bb->SetBool("avoidRobots", false);
	// 	// avoid setting ready continuously in rosparam because it can become quite heavy on ROS
	// 	if (GetBool("setSignal") && !readyHasBeenSet) {
	// 		ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", true);
	// 		readyHasBeenSet = true;
	// 	}
	// } else {
	// 	private_bb->SetBool("avoidRobots", true);
	// 	if (readyHasBeenSet && posError.length() > 0.1 + readyDist) {
	// 		ros::param::set("robot" + std::to_string(robotID) + "/readyToReceiveBall", false);
	// 		readyHasBeenSet = false;
	// 	}
	// }

	// If we should shoot at the goal, we have to determine when the ball is going to reach us, so we can immediately shoot on

	// double role_iterations_per_second = 0.0;
	// ros::param::getCached("role_iterations_per_second", role_iterations_per_second);
	// double timeStep;
	// if (role_iterations_per_second == 0.0) {
	// 	timeStep = 1.0 / 30.0;
	// } else {
	// 	timeStep = 1.0 / role_iterations_per_second;
	// }
	double distanceToBall = (ballPos-myPos).length();
	// if (shootAtGoal) {
	// 	if ((ballPos-receiveBallAtPos).length() < (ballVel.scale(timeStep).length() * 5.0)) {
	// 		startKicking = true;
	// 		ROS_DEBUG_STREAM_NAMED("skills.ReceiveBall", "Start Kicking");
	// 	}
	// }
	if (shootAtGoal && distanceToBall < 2.0) {
		startKicking = true;
	}

	// If the ball gets close, turn on the dribbler
	double dribblerDist = acceptableDeviation * 2.0;
	if (HasDouble("dribblerDist")) {
		dribblerDist = GetDouble("dribblerDist");
	}
	if (shootAtGoal) {
		dribblerDist = 0.0;
	}

	if (distanceToBall < dribblerDist) {
		private_bb->SetBool("dribbler", true);
	} else {
		private_bb->SetBool("dribbler", false);
	}


	// Check the IHaveBall condition to see whether the GetBall skill succeeded
		// auto bb3 = std::make_shared<bt::Blackboard>();
		// bb3->SetInt("me", robotID);
		// bb3->SetBool("our_team", true);
		// IHaveBall iHaveBall2("", bb3);
		// double ballSpeed = Vector2(world.ball.vel.x, world.ball.vel.y).length();
	    // if (iHaveBall2.Update() == Status::Success && ballSpeed < 0.1) {
	    // if (iHaveBall2.Update() == Status::Success) {


	double angleError = cleanAngle(targetAngle - robot.angle);
	bool matchBallVel = (distanceToBall <= 0.6 && fabs(angleError) <= 0.2)
						&& !shootAtGoal && robotID != blackboard->GetInt("KEEPER_ID");

    if (ballWasComing && !ballIsComing && !GetBool("stayAwayFromBall", false)) {
    // If the ball just stopped coming, success or failure will be returned based on the current distance to the ball
    	if(distanceToBall <= 0.4) {
			RTT_DEBUGLN("ReceiveBall skill completed.");
			publishStopCommand();
			ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "robot " << robotID << " succeeded because ball stopped coming and distanceToBall < 0.4");
			return Status::Success;
    	} else if (!GetBool("defenderMode")) { // && isRobotClosestToBall.Update() == Status::Success
    	// If I'm not defending, i.e. actually expecting a pass but the ball stopped too far away from me, return failure
    		ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "robot " << robotID << " failed because ball stopped coming but distanceToBall > 0.4");
    		return Status::Failure;
    	} else {
    	// If I'm just defending, but the ball stopped too far away from me, return running
    		return Status::Running;
    	}

	} else {

        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        if (!HasBool("pGainLarger") || GetBool("pGainLarger") || ballIsComing) {
        	private_bb->SetBool("pGainLarger", true);
        } else {
        	private_bb->SetBool("pGainLarger", false);
        }
        if (HasString("stayOnSide")) {
        	private_bb->SetString("stayOnSide", GetString("stayOnSide"));
        }
        if (HasBool("stayAwayFromBall") && GetBool("stayAwayFromBall")) {
        	private_bb->SetBool("stayAwayFromBall", true);
        }
        if (HasDouble("maxSpeed")) {
        	private_bb->SetDouble("maxSpeed", GetDouble("maxSpeed"));
        }
        if (HasBool("enterDefenseAreas")) {
        	private_bb->SetBool("enterDefenseAreas", GetBool("enterDefenseAreas"));
    	}
		if (HasBool("avoidRobots")) {
			private_bb->SetBool("avoidRobots", GetBool("avoidRobots"));
		}

        boost::optional<roboteam_msgs::RobotCommand> commandPtr = goToPos.getVelCommand();
	    if (commandPtr) {
	        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	        roboteam_msgs::RobotCommand command = *commandPtr;

	        // TEMPORARY REPLACEMENT OF BALL SENSOR FOR IN GRSIM
	        if (startKicking) {
	        	command.kicker = true;
	        	command.kicker_vel = 6.5;
	        } else {
	        	command.kicker = false;
	        }

	    	//    if (matchBallVel) {
			    //     Vector2 ballVelInRobotFrame = worldToRobotFrame(ballVel, robot.angle).scale(0.5);
			    //     Vector2 newVelCommand(command.x_vel + ballVelInRobotFrame.x, command.y_vel + ballVelInRobotFrame.y);
			    //     if (newVelCommand.length() > 4.0) {
			    //       newVelCommand.scale(4.0 / newVelCommand.length());
			    //     }
			    //     command.x_vel = newVelCommand.x;
			    //     command.y_vel = newVelCommand.y;
			    // }

	        pub.publish(command);
	    } else {
	    	publishStopCommand();
	    }


	    return Status::Running;
	}
};

} // rtt
