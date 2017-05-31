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
// yoooooooo
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
        return bt::Node::Status::Failure;
    }

	roboteam_msgs::World world = LastWorld::get();
    Vector2 currentBallVel(world.ball.vel.x, world.ball.vel.y);

    if ((currentBallVel - oldBallVel).length() > 0.1) {
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

    // if (HasBool("wait") && GetBool("wait")) {
    //     // Check if we have waited for longer than the wait time
    //     double waitTime = 250;
    //     if (HasDouble("waitTime")) {
    //         waitTime = GetDouble("waitTime");
    //     }

    //     if (time_difference_milliseconds(startTime, now()).count() >= waitTime) {
    //         ROS_WARN_STREAM("Kicker timer expired!\n");
    //         return Status::Failure;
    //     }

    //     // Check if the ball has moved beyond the threshold
    //     auto world = LastWorld::get();
    //     Vector2 ballNewPos = world.ball.pos;
    //     double distThreshold = 0.10;
    //     if (HasDouble("distThreshold")) {
    //         distThreshold = GetDouble("distThreshold");
    //     }

    //     if ((ballNewPos - ballStartPos).length() >= distThreshold) {
    //         ROS_WARN_STREAM("Ball moved beyond threshold, succeeded!");
    //         return Status::Success;
    //     }

    //     // Otherwise kick the mofo
    //     roboteam_msgs::RobotCommand command;
    //     command.id = robotID;
    //     command.dribbler = true;
    //     command.kicker = true;
    //     command.kicker_forced = true;
    //     command.kicker_vel = kickVel;
    //     command.x_vel = 0.0;
    //     command.y_vel = 0.0;
    //     command.w = 0.0;

    //     rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);

    //     return Status::Running;
    // }

	// roboteam_msgs::WorldBall ball = world.ball;
// yoooooooo
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
        
	RTT_DEBUGLN("Triggered the kicker!");
	return Status::Running;

}

} // rtt