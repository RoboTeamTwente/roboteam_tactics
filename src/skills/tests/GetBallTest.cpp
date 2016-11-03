#include "ros/ros.h"

#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/GetBall.h"
#include "roboteam_msgs/World.h"

bool success;

void msgCallBackGetBall(const roboteam_msgs::WorldConstPtr& world, rtt::GetBall* getBall_B) {
	rtt::LastWorld::set(*world);
	if (getBall_B->Update() == bt::Node::Status::Success) {
		success = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "GetBallTest");
	ros::NodeHandle n;

	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetInt("ROBOT_ID", 1);
	bb->SetBool("intercept", false);
	bb->SetDouble("getBallAtX", 0.0);
	bb->SetDouble("getBallAtY", 0.0);
	bb->SetDouble("getBallAtTime", 5.0);

	rtt::GetBall getBall_B(n, "", bb);
	
	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackGetBall, _1, &getBall_B));

	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			break;
		}
	}
	ROS_INFO("GetBall Test completed");

	return 0;
}
