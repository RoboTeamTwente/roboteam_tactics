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

void msgCallBackKickSkill(const roboteam_msgs::WorldConstPtr& world, rtt::GetBall* getBall) {
	rtt::LastWorld::set(*world);
	if (state == 1) {
		if (getBalll->Update() != bt::Node::Status::Running) {
			finished = true;
		}
	}

	if (state == 2) {
		if (getBall->Update() != bt::Node::Status::Running) {
			finished = true;
		}
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "KickSkillTest");
	ros::NodeHandle n;

	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetInt("ROBOT_ID", 0);
	
	rtt::KickSkill kickSkill(n, "", bb);
	rtt::GetBall getBall(n, "", bb);
	// rtt:RotpointSkill RotpointSkill(n, "", bb);

	int robotID = 0;

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackKickSkill, _1, &getBall));

	while (ros::ok()) {
		ros::spinOnce();
		if (finished) {
			finished = false;
			state = 2;
			break;
		}
	}
	ROS_INFO("KickSkill Test completed!");

	return 0;
}
