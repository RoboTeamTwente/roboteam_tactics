#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class TestSkill : public Skill {

private: 
	roboteam_utils::Vector2 prevTargetPos;
	double prevTargetAngle;
	roboteam_msgs::RobotCommand prevCommand;
	ros::NodeHandle n;
	ros::Publisher pubTestSkill;
	GoToPos goToPos;
public:

	TestSkill(ros::NodeHandle nh) : 
			Skill{aggregator} {	
				n = nh;
				goToPos.Initialize(n, 0);
				pubTestSkill = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
				ROS_INFO("TestSkill constructor");
	}

	Status Update (){
		// ROS_INFO("update");
		roboteam_msgs::World world = LastWorld::get();
		while (world.robots_yellow.size() == 0) {
			return Status::Running;
		}
		roboteam_msgs::WorldBall ball = world.ball;
		roboteam_msgs::WorldRobot robot = world.robots_yellow.at(0);

		roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
		roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
		roboteam_utils::Vector2 posDiff = ballPos-robotPos;
		// Robot radius = 0.09
		roboteam_utils::Vector2 posDiffNorm = posDiff.normalize();
		roboteam_utils::Vector2 targetPos = ballPos - posDiffNorm.scale(0.09);
		double targetAngle = posDiff.angle();

		// roboteam_msgs::SteeringGoal goal;
		// goal.x = targetPos.x;
		// goal.y = targetPos.y;
		// goal.orientation = targetAngle;
		// ROS_INFO_STREAM(posDiff.length());

		if (posDiff.length() < 0.105) {
			ROS_INFO("Target position reached, starting dribbler...");
			roboteam_msgs::RobotCommand command;
			command.x_vel = 0.0;
			command.y_vel = 0.0;
			command.w_vel = 0.0;
			command.dribbler = true;
			pubTestSkill.publish(command);
			ros::spinOnce();
			return Status::Success;
		} else {
			if (fabs(prevTargetPos.x-targetPos.x) > 0.03 || fabs(prevTargetPos.y-targetPos.y) > 0.03 || fabs(prevTargetAngle-targetAngle) > 0.03) {
				goToPos.UpdateArgs(targetPos.x, targetPos.y, targetAngle);
				// ROS_INFO_STREAM(goal);
			}
			goToPos.Update();
			// oldGoal = goal;
			prevTargetPos = roboteam_utils::Vector2(targetPos.x, targetPos.y);
			prevTargetAngle = targetAngle;
			return Status::Running;
		}
	}
};

} // rtt

// bool success;

// void msgCallBackTestSkill(const roboteam_msgs::WorldConstPtr& world, rtt::TestSkill* testSkill) {
// 	rtt::LastWorld::set(*world);
// 	if (testSkill->Update() == bt::Node::Status::Success) {
// 		success = true;
// 	}
// }

// int main(int argc, char **argv) {
// 	ros::init(argc, argv, "TestSkill");
// 	ros::NodeHandle n;
// 	rtt::TestSkill testSkill(n);
// 	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackTestSkill, _1, &testSkill));
// 	// ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBackTestSkill);
// 	// <roboteam_msgs::World>

// 	while (ros::ok()) {
// 		ros::spinOnce();
// 		if (success) {
// 			break;
// 		}
// 	}
// 	ROS_INFO("Skill completed");

// 	return 0;
// }