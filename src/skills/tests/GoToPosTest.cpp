#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

void msgCallBackGoToPos(const roboteam_msgs::World world) {
	rtt::LastWorld::set(world);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "GoToPosTest");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBackGoToPos);

	double xGoal = 1;
	double yGoal = -1;
	double wGoal = 2;
	int robotID = 0;
	rtt::Aggregator aggregator;
	ROS_INFO("Setup done");
	rtt::GoToPos goToPos;
	goToPos.Initialize(n, robotID);
	goToPos.UpdateArgs(xGoal, yGoal, wGoal);

	while (ros::ok()) {
		ros::spinOnce();
		if (goToPos.Update() == bt::Node::Status::Success) {
			break;
		}
	}
	return 0;
}