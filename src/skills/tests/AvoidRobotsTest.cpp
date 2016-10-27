#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/AvoidRobots.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

bool finished1;
bool finished2;
bool finished3;

void msgCallBackAvoidRobots(const roboteam_msgs::WorldConstPtr& world, rtt::AvoidRobots* avoidRobots1) {
	rtt::LastWorld::set(*world);
	if (!finished1) {
		if (avoidRobots1->Update() != bt::Node::Status::Running) {
			finished1 = true;
		}
	}
}

// void msgCallBackAvoidRobots(const roboteam_msgs::WorldConstPtr& world, rtt::AvoidRobots* avoidRobots1, rtt::AvoidRobots* avoidRobots2, rtt::AvoidRobots* avoidRobots3) {
// 	rtt::LastWorld::set(*world);
// 	if (!finished1) {
// 		if (avoidRobots1->Update() != bt::Node::Status::Running) {
// 			finished1 = true;
// 		}
// 	}
// 	if (!finished2) {
// 		if (avoidRobots2->Update() != bt::Node::Status::Running) {
// 			finished2 = true;
// 		}
// 	}
// 	if (!finished3) {
// 		if (avoidRobots3->Update() != bt::Node::Status::Running) {
// 			finished3 = true;
// 		}
// 	}
// }

int main(int argc, char **argv) {
	ros::init(argc, argv, "AvoidRobotsTest");
	ros::NodeHandle n;

	auto bb1 = std::make_shared<bt::Blackboard>();
	bb1->SetInt("ROBOT_ID", 0);
	bb1->SetDouble("targetX", 0.0);
	bb1->SetDouble("targetY", 0.0);
	// auto bb2 = std::make_shared<bt::Blackboard>();
	// bb2->SetInt("ROBOT_ID", 1);
	// bb2->SetDouble("targetX", 1.0);
	// bb2->SetDouble("targetY", -2.0);
	// auto bb3 = std::make_shared<bt::Blackboard>();
	// bb3->SetInt("ROBOT_ID", 2);
	// bb3->SetDouble("targetX", -2.0);
	// bb3->SetDouble("targetY", 0.0);

	rtt::AvoidRobots avoidRobots1(n, "", bb1);
	// rtt::AvoidRobots avoidRobots2(n, "", bb2);
	// rtt::AvoidRobots avoidRobots3(n, "", bb3);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackAvoidRobots, _1, &avoidRobots1));
	// ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackAvoidRobots, _1, &avoidRobots1, &avoidRobots2, &avoidRobots3));

	while (ros::ok()) {
		ros::spinOnce();
		// if (finished1 && finished2 && finished3) {
		// 	break;
		// }
		if (finished1) {
			break;
		}
	}
	ROS_INFO("AvoidRobots Test completed!");

	return 0;
}
