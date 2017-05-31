#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Kick.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG Kick

namespace rtt {

RTT_REGISTER_SKILL(Kick);

Kick::Kick(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) { }

void Kick::Initialize() {
    auto vel = LastWorld::get().ball.vel;
    oldBallVel = Vector2(vel.x, vel.y);
    cycleCounter = 0;

    ballStartPos = LastWorld::get().ball.pos;
    startTime = now();
}

bt::Node::Status Kick::Update() {
  	// ROS_INFO_STREAM("Kick Update");
    if (HasBool("wait_for_signal")) {
    	if (GetBool("wait_for_signal")) {
    		bool readyToPass = false;
    		ros::param::get("readyToReceiveBall", readyToPass);
    		if (!readyToPass) {
    			return Status::Running;
    		}
    	}
    }

    cycleCounter++;

    if (cycleCounter > 20) {
        // ROS_INFO_STREAM("Failed to kick!\n");
        return bt::Node::Status::Failure;
    }

	roboteam_msgs::World world = LastWorld::get();
	
    Vector2 currentBallVel(world.ball.vel.x, world.ball.vel.y);

    // RTT_DEBUG("Velocity difference is %f\n", (currentBallVel - oldBallVel).length());
    if ((currentBallVel - oldBallVel).length() > 0.1) {
        // ROS_INFO_STREAM("Velocity difference was enough\n");
        return bt::Node::Status::Success;
    }

    oldBallVel = currentBallVel;

    int robotID = blackboard->GetInt("ROBOT_ID");
    
    double kickVel;
    if (HasDouble("kickVel")) {
    	kickVel = GetDouble("kickVel");
    } else {
    	kickVel = 4;
    }

    if (HasBool("wait") && GetBool("wait")) {
        // Check if we have waited for longer than the wait time
        double waitTime = 250;
        if (HasDouble("waitTime")) {
            waitTime = GetDouble("waitTime");
        }

        if (time_difference_milliseconds(startTime, now()).count() >= waitTime) {
            ROS_WARN_STREAM("Kicker timer expired!\n");
            return Status::Failure;
        }

        // Check if the ball has moved beyond the threshold
        auto world = LastWorld::get();
        Vector2 ballNewPos = world.ball.pos;
        double distThreshold = 0.10;
        if (HasDouble("distThreshold")) {
            distThreshold = GetDouble("distThreshold");
        }

        if ((ballNewPos - ballStartPos).length() >= distThreshold) {
            ROS_WARN_STREAM("Ball moved beyond threshold, succeeded!");
            return Status::Success;
        }

        // Otherwise kick the mofo
        roboteam_msgs::RobotCommand command;
        command.id = robotID;
        command.dribbler = true;
        command.kicker = true;
        command.kicker_forced = true;
        command.kicker_vel = kickVel;
        command.x_vel = 0.0;
        command.y_vel = 0.0;
        command.w = 0.0;

        rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);

        return Status::Running;
    }

	// ROS_INFO_STREAM("name: " << name << " " << robotID);
	roboteam_msgs::WorldBall ball = world.ball;

	// roboteam_msgs::WorldRobot robot;
    
 //    if (auto botOpt = getWorldBot(robotID)) {
 //        robot = *botOpt;
 //    } else {
 //        RTT_DEBUGLN("Could not lookup our bot %d", robotID);
 //        return Status::Failure;
 //    }

	// Vector2 ballPos = Vector2(ball.pos.x, ball.pos.y);
	// Vector2 robotPos = Vector2(robot.pos.x, robot.pos.y);
	// Vector2 posDiff = ballPos-robotPos;

	// double rotDiff = posDiff.angle() - robot.angle;
	// rotDiff = cleanAngle(rotDiff);
 //    double const rotDiffErr = 0.2;

	// if (posDiff.length() < 0.11) { // ball is close
		// if(rotDiff < rotDiffErr and rotDiff > -rotDiffErr){ // ball in front

    // #SUPERUGLY

    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.dribbler = true;
    command.kicker = true;
    command.kicker_forced = true;
    command.kicker_vel = kickVel;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;

    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);

    // command.dribbler = false;
    // command.kicker = false;
    // command.kicker_forced = false;
    // command.kicker_vel = 0.0;
    // command.x_vel = 0.0;
    // command.y_vel = 0.0;
    // command.w = 0.0;

    // rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
        
	RTT_DEBUGLN("Triggered the kicker!");
	return Status::Running;

}

} // rtt
