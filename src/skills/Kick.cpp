#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/Math.h"

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
    oldBallVel = roboteam_utils::Vector2(vel.x, vel.y);
    cycleCounter = 0;
}

bt::Node::Status Kick::Update() {
  	
    if (HasBool("wait_for_signal")) {
    	if (GetBool("wait_for_signal")) {
    		bool readyToPass = false;
    		ros::param::get("ready_to_pass", readyToPass);
    		if (!readyToPass) {
    			return Status::Running;
    		}
    	}
    }

    cycleCounter++;

    if (cycleCounter > 20) {
        RTT_DEBUG("Failed to kick!\n");
        return bt::Node::Status::Failure;
    }

	roboteam_msgs::World world = LastWorld::get();
	
	
    roboteam_utils::Vector2 currentBallVel(world.ball.vel.x, world.ball.vel.y);
    
    RTT_DEBUG("Velocity difference is %f", (currentBallVel - oldBallVel).length());
    if ((currentBallVel - oldBallVel).length() > 0.1) {
        RTT_DEBUG("Velocity difference was enough\n");
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

	// ROS_INFO_STREAM("name: " << name << " " << robotID);
	roboteam_msgs::WorldBall ball = world.ball;

	// Check is world contains a sensible message. Otherwise wait, it might the case that GoToPos::Update
	// is called before the first world state update
	if (world.us.size() == 0) {
		RTT_DEBUG("No information about the world state :(\n");
		return Status::Running;
	}

	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	roboteam_utils::Vector2 posDiff = ballPos-robotPos;

	double rotDiff = posDiff.angle() - robot.angle;
	rotDiff = cleanAngle(rotDiff);

	if (posDiff.length() < 0.105) { // ball is close

		if(rotDiff < 0.2 and rotDiff > -0.2){ // ball in front
			roboteam_msgs::RobotCommand command;
			command.id = robotID;
			command.dribbler = false;
			command.kicker = true;
			command.kicker_forced = true;
			command.kicker_vel = kickVel;
			command.x_vel = 0.0;
			command.y_vel = 0.0;
			command.w = 0.0;

            rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
            // TODO: Shouldn't be needed
			ros::spinOnce();

			RTT_DEBUG("Triggered the kicker!\n");
			return Status::Running;
		}
		else {
			RTT_DEBUG("Ball is not in front of the dribbler\n");
			return Status::Failure;
		}
	}
	else {
		RTT_DEBUG("Ball is not close to the robot\n");
		return Status::Failure;
	}
}

} // rtt
