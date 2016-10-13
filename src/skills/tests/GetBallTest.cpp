#include "ros/ros.h"

#include "roboteam_msgs/World.h"

bool success;

void msgCallBackTestSkill(const roboteam_msgs::WorldConstPtr& world, rtt::TestSkill* testSkill) {
	rtt::LastWorld::set(*world);
	if (testSkill->Update() == bt::Node::Status::Success) {
		success = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "TestSkill");
	ros::NodeHandle n;
	rtt::TestSkill testSkill(n);
	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackTestSkill, _1, &testSkill));
	// ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBackTestSkill);
	// <roboteam_msgs::World>

	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			break;
		}
	}
	ROS_INFO("Skill completed");

	return 0;
}