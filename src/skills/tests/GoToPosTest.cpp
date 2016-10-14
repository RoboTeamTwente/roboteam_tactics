#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

bool success;

void msgCallBackGoToPos(const roboteam_msgs::WorldConstPtr& world, rtt::GoToPos* goToPos) {
	rtt::LastWorld::set(*world);
	if (goToPos->Update() == bt::Node::Status::Success) {
		success = true;
	}
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "GoToPosTest");
	ros::NodeHandle n;

	double xGoal = 1;
	double yGoal = -1;
	double wGoal = 2;
	int robotID = 0;
	bool endPoint = false;
	rtt::GoToPos goToPos;
	goToPos.Initialize(n, robotID);
	goToPos.UpdateArgs(xGoal, yGoal, wGoal, endPoint);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackGoToPos, _1, &goToPos));

	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			break;
		}
	}
	ROS_INFO("GoToPos Test completed!");
	return 0;
}