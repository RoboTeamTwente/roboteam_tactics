#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/StandFree.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

bool finished1;

void msgCallBack(const roboteam_msgs::WorldConstPtr& world, rtt::StandFree* standFree) {
	rtt::LastWorld::set(*world);
	if (!finished1) {
		if (standFree->Update() != bt::Node::Status::Running) {
			finished1 = true;
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "StandFreeTest");
	ros::NodeHandle n;

	auto bb1 = std::make_shared<bt::Blackboard>();
	bb1->SetInt("ROBOT_ID", 0);
	bb1->SetInt("theirID", 1);
	bb1->SetString("whichTeam", "us");

	rtt::StandFree standFree(n, "", bb1);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBack, _1, &standFree));

	while (ros::ok()) {
		ros::spinOnce();
		if (finished1) {
			break;
		}
	}
	ROS_INFO("StandFree Test completed!");

	return 0;
}