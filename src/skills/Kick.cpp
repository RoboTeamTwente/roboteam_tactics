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
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG Kick

namespace rtt {

RTT_REGISTER_SKILL(Kick);

Kick::Kick(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard), goneForward(false) { }

void Kick::Initialize() {
    auto vel = LastWorld::get().ball.vel;
    oldBallVel = Vector2(vel.x, vel.y);
    ballStartPos = LastWorld::get().ball.pos;
}
/*
bt::Node::Status Kick::Update() {
  	// ROS_INFO_STREAM("Kick Update");
 //    if (HasBool("wait_for_signal")) {
 //    	if (GetBool("wait_for_signal")) {
 //    		bool readyToPass = false;
 //    		ros::param::get("ready_to_pass", readyToPass);
 //    		if (!readyToPass) {
 //    			return Status::Running;
 //    		}
 //    	}
 //    }

 //    cycleCounter++;

 //    if (cycleCounter > 20) {
 //        RTT_DEBUG("Failed to kick!\n");
 //        // return bt::Node::Status::Failure;
 //    }

	// roboteam_msgs::World world = LastWorld::get();
	
 //    Vector2 currentBallVel(world.ball.vel.x, world.ball.vel.y);

    // RTT_DEBUG("Velocity difference is %f\n", (currentBallVel - oldBallVel).length());
    // if ((currentBallVel - oldBallVel).length() > 0.01) {
    //     RTT_DEBUG("Velocity difference was enough\n");
    //     return bt::Node::Status::Success;
    // }

    // oldBallVel = currentBallVel;

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
	// roboteam_msgs::WorldBall ball = world.ball;

	// Check is world contains a sensible message. Otherwise wait, it might the case that GoToPos::Update
	// is called before the first world state update
	// if (world.us.size() == 0) {
	// 	RTT_DEBUG("No information about the world state :(\n");
	// 	return Status::Running;
	// }

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
			{
                roboteam_msgs::RobotCommand command;
                command.id = robotID;
                command.dribbler = true;
                command.kicker = true;
                command.kicker_forced = true;
                command.kicker_vel = kickVel;
                command.x_vel = 0.0;
                command.y_vel = 0.0;
                command.w = 0.0;
    // 
                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
            }
            
            {
                roboteam_msgs::RobotCommand command;
                command.id = robotID;
                command.dribbler = false;
                command.kicker = false;
                command.kicker_forced = false;
                command.kicker_vel = 0.0;
                command.x_vel = 0.0;
                command.y_vel = 0.0;
                command.w = 0.0;

                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
                rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);

            }
            
// 
            

			RTT_DEBUGLN("Triggered the kicker!");
			return Status::Success;
		// } else {
			// RTT_DEBUGLN("Ball is not in front of the dribbler");
			// return Status::Failure;
		// }
	// } else {
		// RTT_DEBUGLN("Ball is not close to the robot");
		// return Status::Failure;
	// }
}
*/
bt::Node::Status Kick::Update() {
	if (GetBool("wait_for_signal", false)) {
		RTT_DEBUG("Checking for signal... ");
		bool ready = false;
		ros::param::get("ready_to_pass", ready);
		if (!ready) {
			RTT_DEBUGLN("No...");
			return Status::Running;
		}
		RTT_DEBUGLN("Yes!");
	}
	if (GetBool("pushFirst", false) && !goneForward) {
		goForward();
		RTT_DEBUGLN("Moving forward a bit");
		return Status::Running;
	}
	doKick();
	ros::param::set("ready_to_pass", false);
	return Status::Success;
}

void Kick::doKick() {
	static auto& pub = GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	int id = GetInt("ROBOT_ID");
	for (int i = 0; i < 10; i++) {
		RTT_DEBUGLN("Kicking");
		roboteam_msgs::RobotCommand command;
		command.id = id;
		command.dribbler = false;
		command.kicker = true;
		command.kicker_forced = false;//GetBool("forced", false);
		command.kicker_vel = GetDouble("kickVel", 4.0);
		command.x_vel = 0.0;
		command.y_vel = 0.0;
		command.w = 0.0;
		pub.publish(command);
	}
}

void Kick::goForward() {
	static auto& pub = GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	Vector2 forward(.5, 0);
	forward = forward.rotate(getWorldBot(robotID)->angle);
	roboteam_msgs::RobotCommand command;
	command.id = robotID;
	command.dribbler = true;
	command.kicker = false;
	command.kicker_forced = false;
	command.kicker_vel = 0.0;
	command.x_vel = forward.x;
	command.y_vel = forward.y;
	command.w = 0.0;
	pub.publish(command);
	goneForward = true;
}

} // rtt
