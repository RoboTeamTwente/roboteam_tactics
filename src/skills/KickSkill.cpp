#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/KickSkill.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"


namespace rtt {

KickSkill::KickSkill(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard) { }

void KickSkill::Initialize(int robotIDInput) {
	pubKickSkill = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
	robotID = robotIDInput;
}

double KickSkill::cleanAngle(double angle){
	if (angle < M_PI){
		return fmod(angle-M_PI, (2*M_PI))+M_PI;
	}
	else if(angle > M_PI){
		return fmod(angle+M_PI, (2*M_PI))-M_PI;
	}
	else {
		return angle;
	}
}

bt::Node::Status KickSkill::Update() {
	roboteam_msgs::World world = LastWorld::get();
	roboteam_msgs::WorldBall ball = world.ball;

	// Check is world contains a sensible message. Otherwise wait, it might the case that GoToPos::Update 
	// is called before the first world state update
	if (world.robots_yellow.size() == 0) {
		ROS_INFO("No information about the world state :(");
		return Status::Running;
	}

	roboteam_msgs::WorldRobot robot = world.robots_yellow[robotID];
	roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	roboteam_utils::Vector2 posDiff = ballPos-robotPos;		

	double rotDiff = posDiff.angle() - robot.w;
	rotDiff = cleanAngle(rotDiff);

	if (posDiff.length() < 0.105) { // ball is close
		if(rotDiff < 0.1 and rotDiff > -0.1){ // ball in front
			ROS_INFO("Kicking the ball");
			roboteam_msgs::RobotCommand command;
			command.dribbler = false;
			command.kicker = true;
			command.kicker_forced = true;
			command.kicker_vel = 2;
			command.x_vel = 0.0;
			command.y_vel = 0.0;
			command.w_vel = 0.0;

			pubKickSkill.publish(command);
			return Status::Success;
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
