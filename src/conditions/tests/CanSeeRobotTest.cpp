#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/CanSeeTheirGoal.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_utils/Vector2.h"

bool success;
bool failure;

void msgCallBack(const roboteam_msgs::WorldConstPtr& world, rtt::CanSeeTheirGoal* canSeeTheirGoal) {
	rtt::LastWorld::set(*world);
	bt::Node::Status status = canSeeTheirGoal->Update();
	if (status == bt::Node::Status::Success) {
		success = true;
	}
	if (status == bt::Node::Status::Failure) {
		failure = true;
	}
}

void fieldUpdateCallback(const roboteam_msgs::GeometryDataConstPtr& geom) {
    rtt::LastWorld::set_field(geom->field);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "CanSeeTheirGoalTest");
	ros::NodeHandle n;

	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetInt("ROBOT_ID", 1);

	rtt::CanSeeTheirGoal canSeeTheirGoal("", bb);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> (TOPIC_WOLRD_STATE, 1000, boost::bind(&msgCallBack, _1, &canSeeTheirGoal));
	ros::Subscriber subField = n.subscribe(TOPIC_GEOMETRY, 10, &fieldUpdateCallback);

	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			ROS_INFO("CanSeeTheirGoalTest completed");
			break;
		}
		if (failure) {
			ROS_INFO("CanSeeTheirGoalTest failed :(");
			break;
		}
	}
	return 0;
}