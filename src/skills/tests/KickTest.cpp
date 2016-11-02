#include "ros/ros.h"

#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Kick.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

bool finished;

void msgCallBackKick(const roboteam_msgs::WorldConstPtr& world, rtt::Kick* Kick) {
	rtt::LastWorld::set(*world);
	if (Kick->Update() != bt::Node::Status::Running) {
		finished = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "KickTest");
	ros::NodeHandle n;

	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetInt("ROBOT_ID", 1);

	rtt::Kick Kick(n, "", bb);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackKick, _1, &Kick));

	while (ros::ok()) {
		ros::spinOnce();
		if (finished) {
			break;
		}
	}
	ROS_INFO("Kick Test completed!");

	return 0;
}
