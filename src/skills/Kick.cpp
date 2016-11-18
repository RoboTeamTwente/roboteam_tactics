#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Kick.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

Kick::Kick(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard) {
        	pubKick = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
            // ROS_INFO("Kicking the ball");
}

double Kick::cleanAngle(double angle) {
	if (angle < M_PI){
		return fmod(angle-M_PI, (2*M_PI))+M_PI;
	} else if(angle > M_PI){
		return fmod(angle+M_PI, (2*M_PI))-M_PI;
	} else {
		return angle;
	}
}

void Kick::Initialize() {
    auto vel = LastWorld::get().ball.vel;
    oldBallVel = roboteam_utils::Vector2(vel.x, vel.y);
    cycleCounter = 0;
}

bt::Node::Status Kick::Update() {
    cycleCounter++;
    if (cycleCounter > 10) return bt::Node::Status::Failure;

	roboteam_msgs::World world = LastWorld::get();

    roboteam_utils::Vector2 currentBallVel(world.ball.vel.x, world.ball.vel.y);

    if ((currentBallVel - oldBallVel).length() >= 0.5) {
        ROS_INFO("Velocity difference was enough");
        return bt::Node::Status::Success;
    }

    oldBallVel = currentBallVel;

    int robotID = blackboard->GetInt("ROBOT_ID");
	// ROS_INFO_STREAM("name: " << name << " " << robotID);
	roboteam_msgs::WorldBall ball = world.ball;

	// Check is world contains a sensible message. Otherwise wait, it might the case that GoToPos::Update 
	// is called before the first world state update
	if (world.us.size() == 0) {
		ROS_INFO("No information about the world state :(");
		return Status::Running;
	}

	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	roboteam_utils::Vector2 posDiff = ballPos-robotPos;		

	double rotDiff = posDiff.angle() - robot.angle;
	rotDiff = cleanAngle(rotDiff);

	if (posDiff.length() < 0.105) { // ball is close
		if(rotDiff < 0.1 and rotDiff > -0.1){ // ball in front
			roboteam_msgs::RobotCommand command;
			command.id = robotID;
			command.dribbler = false;
			command.kicker = true;
			command.kicker_forced = true;
			command.kicker_vel = 5;
			command.x_vel = 0.0;
			command.y_vel = 0.0;
			command.w = 0.0;

			pubKick.publish(command);
			ros::spinOnce();
			ROS_INFO("Triggered the kicker!");
			return Status::Running;
		}
		else {
			ROS_INFO("Ball is not in front of the dribbler");
			return Status::Failure;
		}
	}
	else {
		ROS_INFO("Ball is not close to the robot");
		return Status::Failure;
	} 
}

} // rtt
