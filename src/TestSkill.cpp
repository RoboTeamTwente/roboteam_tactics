#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
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

public:
	TestSkill(Aggregator& aggregator) : 
			Skill{aggregator} {
	}

	Status Update (){
		roboteam_msgs::World world = LastWorld::get();
		roboteam_msgs::WorldBall ball = world.ball;
		roboteam_msgs::WorldRobot robot = world.robots_yellow[0];

		roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
		roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
		roboteam_utils::Vector2 posDiff = ballPos.operator-(robotPos);
		roboteam_utils::Vector2 targetPos = ballPos.operator-(posDiff.scale(posDiff.length()*0.09));
		double targetAngle = posDiff.angle();

		roboteam_msgs::SteeringGoal goal;
		goal.x = targetPos.x;
		goal.y = targetPos.y;
		goal.orientation = targetAngle;

		if (posDiff.length() < 0.1) {
			goal.dribbler = true;
			return Status::Success;
		} else {
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

	rtt::Aggregator aggregator;
	rtt::TestSkill testSkill(aggregator);
	while (testSkill.Update() == bt::Node::Status::Running) {
		ros::spinOnce();
	}

	return 0;
}
