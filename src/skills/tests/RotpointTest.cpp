#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/RotpointSkill.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_utils/Vector2.h"

void msgCallBack(const roboteam_msgs::World world) {
	rtt::LastWorld::set(world);
	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "RotpointSkill");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBack);

	rtt::RotpointSkill rotpointSkill(n);
	ros::Publisher pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
	// TODO: advertising on publisher is slow	
	ROS_INFO("waiting for connection");
	while(pub.getNumSubscribers() < 1){sleep(0.1);}
	ROS_INFO("connected");
	rotpointSkill.updateArgs(pub,0,0,2.0);
	while (ros::ok()) {
		ros::spinOnce();
		if(rotpointSkill.Update() != bt::Node::Status::Running){break;} // break; breaks the code if only one message is sent
		
	}
	ROS_INFO("Skill completed");
	return 0;
}

