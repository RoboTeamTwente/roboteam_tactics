#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class TestSkill : public Skill {

private: 
	roboteam_msgs::SteeringGoal oldGoal;

public:

	TestSkill(Aggregator& aggregator) : 
			Skill{aggregator} {	
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
		roboteam_utils::Vector2 targetPos = ballPos - posDiffNorm.scale(0.10);
		double targetAngle = posDiff.angle();

		roboteam_msgs::SteeringGoal goal;
		goal.x = targetPos.x;
		goal.y = targetPos.y;
		goal.orientation = targetAngle;

		if (posDiff.length() < 0.105) {
			ROS_INFO("Target position reached, starting dribbler...");
			goal.x = robot.pos.x;
			goal.y = robot.pos.y;
			goal.orientation = robot.w;
			goal.dribbler = true;
			aggregator.putMsg(0, goal);
			return Status::Success;
		} else {
			if (fabs(oldGoal.x-goal.x) > 0.03 || fabs(oldGoal.y-goal.y) > 0.03 || fabs(oldGoal.orientation-goal.orientation) > 0.03) {
				aggregator.putMsg(0, goal);
				ROS_INFO_STREAM(goal);
			}
			oldGoal = goal;
			return Status::Running;
		}

	}
};

} // rtt

void msgCallBack(const roboteam_msgs::World world) {
	rtt::LastWorld::set(world);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "TestSkill");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBack);

	// ROS_INFO("1");

	rtt::Aggregator aggregator;
	rtt::TestSkill testSkill(aggregator);
	while (ros::ok()) {
		ros::spinOnce();
		if (testSkill.Update() == bt::Node::Status::Success) {
			break;
		}
	}
	ROS_INFO("Skill completed");

	return 0;
}
