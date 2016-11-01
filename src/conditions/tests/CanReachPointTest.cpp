#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/CanReachPoint.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

bool success;
bool failure;

void msgCallBack(const roboteam_msgs::WorldConstPtr& world, rtt::CanReachPoint* canReachPoint) {
	rtt::LastWorld::set(*world);
	bt::Node::Status status = canReachPoint->Update();
	if (status == bt::Node::Status::Success) {
		success = true;
	}
	if (status == bt::Node::Status::Failure) {
		failure = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "CanReachPointTest");
	ros::NodeHandle n;

	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetInt("ROBOT_ID", 1);
	bb->SetDouble("xGoal", 0.0);
	bb->SetDouble("yGoal", 0.0);
	bb->SetDouble("timeLimit", 1.0);

	rtt::CanReachPoint canReachPoint("", bb);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBack, _1, &canReachPoint));

	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			ROS_INFO("Yep, you can reach this point");
			break;
		}
		if (failure) {
			ROS_INFO("Nope, you can't reach this point :(");
			break;
		}
	}
	return 0;
}
