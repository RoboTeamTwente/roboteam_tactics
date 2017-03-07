#include <ros/ros.h>


#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Chip.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG Chip

namespace rtt {

RTT_REGISTER_SKILL(Chip);

Chip::Chip(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) {
}

void Chip::Initialize() {
    auto vel = LastWorld::get().ball.vel;
    oldBallVel = Vector2(vel.x, vel.y);
    cycleCounter = 0;
}

bt::Node::Status Chip::Update() {
    cycleCounter++;
    if (cycleCounter > 10) {
        RTT_DEBUGLN("Too many cycles: aborting!");
        return bt::Node::Status::Failure;
    }

	roboteam_msgs::World world = LastWorld::get();

    Vector2 currentBallVel(world.ball.vel.x, world.ball.vel.y);

    if ((currentBallVel - oldBallVel).length() >= 0.1) {
        RTT_DEBUGLN("Velocity difference was enough");
        return bt::Node::Status::Success;
    }

    oldBallVel = currentBallVel;

    int robotID = blackboard->GetInt("ROBOT_ID");
	// ROS_INFO_STREAM("name: " << name << " " << robotID);
	roboteam_msgs::WorldBall ball = world.ball;

	// Check is world contains a sensible message. Otherwise wait, it might the case that GoToPos::Update
	// is called before the first world state update
	if (world.us.size() == 0) {
		return Status::Running;
	}

	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	Vector2 ballPos = Vector2(ball.pos.x, ball.pos.y);
	Vector2 robotPos = Vector2(robot.pos.x, robot.pos.y);
	Vector2 posDiff = ballPos-robotPos;

	double rotDiff = posDiff.angle() - robot.angle;
	rotDiff = cleanAngle(rotDiff);

	if (posDiff.length() < 0.105) { // ball is close
		if(rotDiff < 0.1 and rotDiff > -0.1){ // ball in front
			roboteam_msgs::RobotCommand command;
			command.id = robotID;
            //command.active = true;
			command.dribbler = false;
			command.chipper = true;
			command.chipper_forced = true;
			command.chipper_vel = 5;
			command.x_vel = 0.0;
			command.y_vel = 0.0;
			command.w = 0.0;

            rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
			RTT_DEBUGLN("Triggered the chipper!");
			return Status::Running;
		}
		else {
			RTT_DEBUGLN("Ball is not in front of the dribbler");
			return Status::Failure;
		}
	}
	else {
		RTT_DEBUGLN("Ball is not close to the robot");
		return Status::Failure;
	}
}

} // rtt
