#include "ros/ros.h"

#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/GetBall.h"
#include "roboteam_msgs/World.h"

bool success;

void msgCallBackGetBall(const roboteam_msgs::WorldConstPtr& world, rtt::GetBall* getBall) {
	rtt::LastWorld::set(*world);
	if (getBall->Update() == bt::Node::Status::Success) {
		success = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "GetBallTest");
	ros::NodeHandle n;
	rtt::GetBall getBall(n);
	
	int robotID = 0;

	getBall.UpdateArgs(robotID);
	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackGetBall, _1, &getBall));

	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			break;
		}
	}
	ROS_INFO("GetBall Test completed");

	return 0;
}