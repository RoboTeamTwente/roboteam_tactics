#include "ros/ros.h"

#include "roboteam_tactics/skills/AvoidRobots.h"
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
        , avoidRobots("", private_bb) {
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
	roboteam_utils::Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();

	double timeToLookIntoFuture = 5.0; // in seconds
	roboteam_utils::Vector2 ballPosNow(world.ball.pos.x, world.ball.pos.y);
	roboteam_utils::Vector2 ballPosThen(LastWorld::predictBallPos(timeToLookIntoFuture));
	roboteam_utils::Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
	roboteam_utils::Vector2 ballTrajectory = ballPosThen - ballPosNow;
	roboteam_utils::Vector2 ballToCenter = receiveBallAtPos - ballPosNow;

	double ballTrajectoryMagn = ballTrajectory.length();
	ballTrajectory = ballTrajectory.scale(1/ballTrajectoryMagn);
	double projectionOnBallTrajectory = ballToCenter.dot(ballTrajectory);
	roboteam_utils::Vector2 closestPoint = ballPosNow + ballTrajectory*projectionOnBallTrajectory;

	// If the ball is moving (towards us) and the computed closest point is within range, then stand on the closest point
	bool ballMovingTowardsUs = (ballPosThen-ballPosNow).dot(receiveBallAtPos-ballPosNow) >= 0;
	if (ballMovingTowardsUs && ballTrajectoryMagn > 0.1 && (closestPoint - receiveBallAtPos).length() < acceptableDeviation) {
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
	roboteam_utils::Vector2 interceptPos;
	double interceptAngle;
	roboteam_msgs::World world = LastWorld::get();

	roboteam_msgs::WorldRobot otherRobot;
	if (our_team) {
		otherRobot = world.us.at(hasBall);
	} else {
		otherRobot = world.them.at(hasBall);
	}

	// If the other robot would shoot now, use its orientation to estimate the ball trajectory, and then the closest
	// point on this trajectory to our robot, so he can receive the ball there
	roboteam_utils::Vector2 otherRobotLooksAt = roboteam_utils::Vector2(1, 0);
	otherRobotLooksAt = otherRobotLooksAt.rotate(otherRobot.angle);
	roboteam_utils::Vector2 ballPosNow(world.ball.pos.x, world.ball.pos.y);
	roboteam_utils::Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
	roboteam_utils::Vector2 ballToCenter = receiveBallAtPos - ballPosNow;
	double projectionOnBallTrajectory = ballToCenter.dot(otherRobotLooksAt);
	roboteam_utils::Vector2 closestPoint = ballPosNow + otherRobotLooksAt*projectionOnBallTrajectory;

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
	int robotID = blackboard->GetInt("ROBOT_ID");
	if (HasDouble("acceptableDeviation")) {
		acceptableDeviation = GetDouble("acceptableDeviation");
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
	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	double robotAngle = robot.angle;
	double distanceToBall = (ballPos-robotPos).length();
	roboteam_utils::Vector2 targetPos;
	double targetAngle;


	// Read the blackboard info about where to receive the ball 
	double receiveBallAtX;
	double receiveBallAtY;
	if (GetBool("receiveBallAtCurrentPos")) {
		receiveBallAtX = robotPos.x;
		receiveBallAtY = robotPos.y;
	} else {
		receiveBallAtX = GetDouble("receiveBallAtX");
		receiveBallAtY = GetDouble("receiveBallAtY");
	}


	// Calculate where we can receive the ball close to the given receiveBallAt... point.
	InterceptPose interceptPose;
	if (hasBall == -1) {
		interceptPose = deduceInterceptPosFromBall(receiveBallAtX, receiveBallAtY);
	} else {
		interceptPose = deduceInterceptPosFromRobot(receiveBallAtX, receiveBallAtY);
	}
	roboteam_utils::Vector2 interceptPos = interceptPose.interceptPos;
	double interceptAngle = interceptPose.interceptAngle;


	// Check whether the point returned by the deduceInterceptPos... function actually returns a point that is close (less than acceptableDeviation)
	// from the specified receiveBallAt... point. This should always be the case (maybe move this check to some test function rather than perform
	// it in the skill update)
	roboteam_utils::Vector2 receiveBallAtPos(receiveBallAtX, receiveBallAtY);
	if (isPointInCircle(receiveBallAtPos, acceptableDeviation, interceptPos)) {
		targetPos = interceptPos;
		targetAngle = interceptAngle;
	} else {
		ROS_ERROR("This is probably not a valid intercept position...");
		return Status::Failure;
	}


	// If we are too far from the ball, or too far from the speficied targetPos, we should drive towards the targetPos
	roboteam_utils::Vector2 posError = targetPos - robotPos;
	if (distanceToBall > acceptableDeviation || posError.length() > acceptableDeviation) {
		double angleError = targetAngle - robotAngle;
		if (posError.length() <= 0.1 && fabs(angleError) < 0.3) {
			// Set a rosparam to let other robots know that we are ready to receive the ball
			ros::param::set("readyToReceiveBall", true);
		}
	} else { 
		// If we are close enough to the ball we can drive towards it and turn on the dribbler
		roboteam_utils::Vector2 posDiff = ballPos - robotPos;
		roboteam_utils::Vector2 posDiffNorm = posDiff.normalize();
		targetPos = ballPos - posDiffNorm.scale(0.09); // 0.09 = robot radius
		targetAngle = posDiff.angle();
		private_bb->SetBool("dribbler", true);
	}


	// Check the IHaveBall condition to see whether the GetBall skill succeeded
	auto bb3 = std::make_shared<bt::Blackboard>();
	bb3->SetInt("me", robotID);
	bb3->SetBool("our_team", true);
	IHaveBall iHaveBall2("", bb3);
    bool doIHaveBall = iHaveBall2.Update() == Status::Success;

    // @Hack this feels like a one-off fix (since this does not always affect ReceiveBall)
    // If I have the ball and I touched it before
    if (doIHaveBall && touchedBall) {
        bool enoughTimeHasPassed = time_difference_milliseconds(initialBallContact, now()).count() > 100;

        // if the angle is 👍 and 100 ms have passed we got the ball!
        if (fabs(targetAngle - robot.angle) < 0.05 && enoughTimeHasPassed) {
            publishStopCommand();
            RTT_DEBUGLN("ReceiveBall skill completed.");
            return Status::Success;
        };

        // else keep avoiding the robots and receiving the ball
    } else if (doIHaveBall && !touchedBall) {
        // If I have the ball but I haven't touched it before
        touchedBall = true;
        initialBallContact = now();
        // It probbaly just arrived, so keep receiving the ball for a bit longer to make
        // sure we really got it.
    }

    // @Incomplete Maybe the result of avoidrobots should be taken into account?
    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetDouble("xGoal", targetPos.x);
    private_bb->SetDouble("yGoal", targetPos.y);
    private_bb->SetDouble("angleGoal", targetAngle);
    private_bb->SetBool("isKeeper", GetBool("isKeeper"));
    avoidRobots.Update();

    return Status::Running;
};

} // rtt
