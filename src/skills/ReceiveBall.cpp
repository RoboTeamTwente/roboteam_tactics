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
        {
    hasBall = whichRobotHasBall();
    ros::param::get("robot_output_target", robot_output_target);
    prevComputedPoint = now();
    opportunityFinder.Initialize("jelle.txt", robotID, "theirgoal", 0);

}

void ReceiveBall::Initialize() {
	hasTerminated = false;
	startTime = now();
	robotID = blackboard->GetInt("ROBOT_ID");
	ROS_INFO_STREAM_NAMED("skills.ReceiveBall", "Initialize for robot: " << robotID);
	ballIsComing = false;
	startKicking = false;

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
		ros::param::get("robot" + std::to_string(robotID) + "/claimedPosX", botClaimedX);
		ros::param::get("robot" + std::to_string(robotID) + "/claimedPosY", botClaimedY);
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
		publishStopCommand();

		if (GetBool("setSignal")) {
		// Check if the robot actually expects to receive a pass
		// If setSignal is not used, the rosparam robotClaimedBall will not be set to my ID (it shouldnt, at least)
		// setSignal refers to the setting of the 'readyToReceiveBall' rosparam
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
		        ROS_INFO_STREAM_NAMED("skills.ReceiveBall", robotID << ", resetting passToRobot to -1 due to Terminate (passToRobot was set to my id)");
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
	command.geneva_state = 3;

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
	double velThreshold;
	if (ballIsComing) {
		velThreshold = 0.5;//posdiff.length()/8; // ARBITRARY GUESS
	} else {
		velThreshold = 0.8;//posdiff.length()/4; // ARBITRARY GUESS
	}
	if (GetBool("defenderMode")) {
		velThreshold = 0.1;
	} else if (velThreshold < 0.1) {
		velThreshold = 0.1;
	}

	static int slowBallCounter = 0;
	if (ballVel.dot(posdiff.normalize()) < velThreshold) { // if the velocity in my direction is too low
		slowBallCounter++;
		if (slowBallCounter > 3) {
			if(!ballIsComing && !GetBool("defenderMode") && GetBool("setSignal")) {
				int robotClaimedBall;
			    ros::param::getCached("robotClaimedBall", robotClaimedBall);
			    if (robotClaimedBall == robotID) { // if a pass is noted to myself, I assume the ball is coming.
			    	ballIsComing = true;
		    		ROS_DEBUG_STREAM_NAMED("skills.ReceiveBall", robotID << " noted a pass to himself, and no teammate has ball, so assumes ball is coming");
			    }
			} else {
				ballIsComing = false;
			}
		}
	} else {
		slowBallCounter = 0;
	}
	//--------------------------

	// Actual determining of the interceptPose
	if (ballVel.length() < 0.1 || ballVel.dot(posdiff.normalize()) < velThreshold) { // ball not coming towards me.
		interceptPose.interceptPos = receiveBallAtPos;
		interceptPose.interceptAngle = (ballPos - receiveBallAtPos).angle();
	} else {
		Vector2 closestPoint = ballPos + (receiveBallAtPos - ballPos).project2(ballVel);
		ballIsComing = true;

		double deviation = (closestPoint - receiveBallAtPos).length();
		if (deviation < acceptableDeviation + marginDeviation) {
			if (deviation < acceptableDeviation) {
				interceptPos = closestPoint;
			} else {
				interceptPos = receiveBallAtPos + (closestPoint - receiveBallAtPos).stretchToLength(acceptableDeviation);
			}

			interceptAngle = cleanAngle(ballVel.angle() + M_PI);
			if (avoidBall) {
				interceptPos = closestPoint + (closestPoint - receiveBallAtPos).normalize();
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
	if (GetBool("defenderMode") && deviation > 0.001 && deviation < acceptableDeviation + marginDeviation) {
		// if the deviation is within the margin region, it should remain at acceptableDeviation until it exceeds the margin
		deviation = fmin(deviation, acceptableDeviation);
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
	double distanceToBall = (ballPos - myPos).length();

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

	// Determine if we should shoot at goal, depending whether the shootAtGoal boolean is set, and on whether we can see the goal
	// and on whether the 'reflection angle' (angleDiff) is small enough
	bool shootAtGoal = false;
	double angleDiff = 0;
	if (GetBool("shootAtGoal")) {
		// we need the last ball position before it was passed to us, because we dont want to change...
		// ...our decision of whether shooting is possible while the ball is already coming
		static Vector2 lastNotComingBall = ballPos; 
		if (!ballIsComing) {
			lastNotComingBall = ballPos;
		}
		double viewOfGoal = opportunityFinder.calcViewOfGoal(receiveBallAtPos, world); // chosen reception pos is used to assess view of goal
		angleDiff = cleanAngle( ((lastNotComingBall - receiveBallAtPos).angle() - (LastWorld::get_their_goal_center() - receiveBallAtPos).angle()) );
		shootAtGoal = (viewOfGoal > 0.1 && fabs(angleDiff) < 0.33*M_PI + 20/180*M_PI); // geneva drive allows 20 degrees larger anglediff
		// ROS_INFO_STREAM_NAMED("skills.ReceiveBall", "viewOfGoal: " << viewOfGoal << ", angleDiff: " << angleDiff << ", shootAtGoal: " << shootAtGoal << ", theirgoal: " << LastWorld::get_their_goal_center());
	}

	// Set the targetAngle, if we should shoot at goal, we should face mostly towards the goal
	int geneva_state = 3; //default geneva drive to center position
	if (shootAtGoal) {
		// decide convenient geneva state
		static double geneva_threshold = 0.25*M_PI;
		if (fabs(angleDiff) > geneva_threshold) {
			geneva_state = (angleDiff<0) ? 5 : 1; // negative anglediff -> geneva to 20 degrees left, otherwise to 20 degrees right
			geneva_threshold = 0.20*M_PI; // prevents rapid shifting of the geneva drive
		} else {
			geneva_state = 3;
			geneva_threshold = 0.25*M_PI;
		}
		// determine shooting angle and take a step back from the ball interception pos
		targetAngle = (LastWorld::get_their_goal_center() - myPos).angle() + (angleDiff / 4.0) - (double)(geneva_state-3)*10/180*M_PI;
		Vector2 robotRadius(0.095, 0.0);
		robotRadius = robotRadius.rotate(targetAngle);
		targetPos = interceptPos - robotRadius;
		startKicking = fabs(cleanAngle(robot.angle - targetAngle)) < 0.2; // extra safety: I shouldnt shoot if I did not manage to reach the right angle in time
	} else {
		targetPos = interceptPos;
		targetAngle = interceptAngle;
		startKicking = false;
	}


	// If the ball hasnt been coming for a while and I'm closest to the ball, or ball is in defense area, return failure.
	if (!ballIsComing) {
		int elapsedTime = time_difference_milliseconds(startTime, now()).count();
		if (elapsedTime > 1000) {
			startTime = now();
			boost::optional<int> robotClosestToBallPtr = get_robot_closest_to_point(world.us, ballPos);
			if ((!GetBool("defenderMode") && robotClosestToBallPtr && *robotClosestToBallPtr == robotID) || (GetBool("defenderMode") && isWithinDefenseArea(true, ballPos, 0.1))) {
				ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "robot " << robotID << " failed because im closest to a ball that hasnt been coming for a while");
				return Status::Failure;
			}
		} 
	} else {
    	startTime = now();
	}

    if (ballWasComing && !ballIsComing && !GetBool("stayAwayFromBall", false)) {
    // If the ball just stopped coming, success or failure will be returned based on the current distance to the ball
    	if(distanceToBall <= 0.4) {
			RTT_DEBUGLN("ReceiveBall skill completed.");
			//publishStopCommand();
			ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "robot " << robotID << " succeeded because ball stopped coming and distanceToBall < 0.4");
			return Status::Success;
    	} else if (!GetBool("defenderMode")) {
    	// If I'm not defending, i.e. actually expecting a pass but the ball stopped too far away from me, return failure
    		ROS_WARN_STREAM_NAMED("skills.ReceiveBall", "robot " << robotID << " failed because ball stopped coming but distanceToBall > 0.4");
    		return Status::Failure;
    	} else {
    	// If I'm just defending, but the ball stopped too far away from me, return running
    		return Status::Running;
    	}

	} else {

		double posErrorLength = (targetPos - myPos).length();
		// new reception strategy because sideways acceleration is very slow for new robots
		static double thresholdSwitch = 1.0;
		double distToBallThresh, turningDist;
		if (GetBool("defenderMode")) {
			distToBallThresh = 0.3;
			turningDist = 0.1;
		} else {
			distToBallThresh = 1.5;
			turningDist = 0.29;
		}
		if (!GetBool("claimedPos") && ((!ballIsComing && distanceToBall > thresholdSwitch*distToBallThresh) ||  (ballIsComing && posErrorLength > turningDist)) ) {
			targetAngle = interceptAngle + M_PI/2;
			thresholdSwitch = 0.95;
		} else {
			thresholdSwitch = 1.0;
		}

		// fill gotopos blackboard
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
        private_bb->SetDouble("xGoal", targetPos.x);
        private_bb->SetDouble("yGoal", targetPos.y);
        private_bb->SetDouble("angleGoal", targetAngle);
        private_bb->SetDouble("successDist", 0.01);
        if (GetBool("pGainLarger") || ballIsComing) {
        	private_bb->SetBool("pGainLarger", true);
        } else {
        	private_bb->SetBool("pGainLarger", false);
        }
        if (HasString("stayOnSide")) {
        	private_bb->SetString("stayOnSide", GetString("stayOnSide"));
        }
        if (blackboard->HasBool("stayAwayFromBall") && blackboard->GetBool("stayAwayFromBall")) {
        	private_bb->SetBool("stayAwayFromBall", true);
        }
        if (HasDouble("maxSpeed")) {
        	private_bb->SetDouble("maxSpeed", GetDouble("maxSpeed"));
        }
        if (blackboard->HasBool("enterDefenseAreas")) {
        	private_bb->SetBool("enterDefenseAreas", blackboard->GetBool("enterDefenseAreas"));
    	}
    	private_bb->SetBool("avoidRobots", true); // posErrorLength > 0.3 // shut off robot avoidance when close to target (SHUT OFF FOR NOW, SEEMS DANGEROUS)
		if (blackboard->HasBool("avoidRobots")) {
			private_bb->SetBool("avoidRobots", blackboard->GetBool("avoidRobots"));
		}
        
        roboteam_msgs::RobotCommand command;
        boost::optional<roboteam_msgs::RobotCommand> commandPtr = goToPos.getVelCommand();
        command.id = robotID;
	    if (commandPtr) {
	        command = *commandPtr;
	    } else {
	    	command.x_vel = 0;
	    	command.y_vel = 0;
	    	command.w = 0;
	    }

	    // Set geneva state asap (never know how fast that ball be coming)
	    command.geneva_state = geneva_state;

        // For a real robot this starts the kicking on ball sensor. For grsim, the robot will continuously kick, giving similar results
        if (startKicking) {
        	command.kicker = true;
        	command.kicker_vel = 6.5;
        } else {
        	command.kicker = false;
        }

    	// If the ball gets close, turn on the dribbler
		double dribblerDist = 2.0;
		if (shootAtGoal) {
			dribblerDist = 0.0;
		}
		if (HasDouble("dribblerDist")) {
			dribblerDist = GetDouble("dribblerDist");
		}
		command.dribbler = distanceToBall < dribblerDist;

		auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        pub.publish(command);

	    return Status::Running;
	}
};

} // rtt
