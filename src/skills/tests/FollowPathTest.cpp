#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/FollowPath.h"
#include "roboteam_utils/Vector2.h"

bool success;

void msgCallBackFollowPath(const roboteam_msgs::WorldConstPtr& world, rtt::FollowPath* followPath) {
	rtt::LastWorld::set(*world);
	if (followPath->Update() == bt::Node::Status::Success) {
		success = true;
	}
}
int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_FOLLOW_PATH_TEST);
	ros::NodeHandle n;

    auto bb = std::make_shared<bt::Blackboard>();
    bb->SetDouble("xGoal", 1);
    bb->SetDouble("yGoal", -1);
    bb->SetDouble("wGoal", 2);
    bb->SetInt("ROBOT_ID", 0);

	rtt::FollowPath followPath("", bb);
	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> (TOPIC_WOLRD_STATE, 1000, boost::bind(&msgCallBackFollowPath, _1, &followPath));

	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			break;
		}
	}
	ROS_INFO("FollowPath Test completed!");
	return 0;
}
