#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/KickSkill.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

bool finished;

void msgCallBackKickSkill(const roboteam_msgs::WorldConstPtr& world, rtt::KickSkill* kickSkill) {
	rtt::LastWorld::set(*world);
	if (kickSkill->Update() != bt::Node::Status::Running) {
		finished = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "KickSkillTest");
	ros::NodeHandle n;

	rtt::KickSkill kickSkill;

	int robotID = 0;

	kickSkill.Initialize(n, robotID);
	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackKickSkill, _1, &kickSkill));

	while (ros::ok()) {
		ros::spinOnce();
		if (finished) {
			break;
		}
	}
	ROS_INFO("KickSkill Test completed!");

	return 0;
}