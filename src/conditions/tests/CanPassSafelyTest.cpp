#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/CanPassSafely.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

bool success;
bool failure;

void msgCallBack(const roboteam_msgs::WorldConstPtr& world, rtt::CanPassSafely* canPassSafely) {
	rtt::LastWorld::set(*world);
	bt::Node::Status status = canPassSafely->Update();
	if (status == bt::Node::Status::Success) {
		success = true;
	}
	if (status == bt::Node::Status::Failure) {
		failure = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "CanPassSafelyTest");
	ros::NodeHandle n;

	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetInt("ROBOT_ID", 0);
	bb->SetInt("passToRobot", 1);

	rtt::CanPassSafely canPassSafely("", bb);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> (TOPIC_WOLRD_STATE, 1000, boost::bind(&msgCallBack, _1, &canPassSafely));

	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			ROS_INFO("CanPassSafelyTest completed");
			break;
		}
		if (failure) {
			ROS_INFO("CanPassSafelyTest failed :(");
			break;
		}
	}
	return 0;
}
