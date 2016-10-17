#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"
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
	ros::init(argc, argv, "RotateAroundPoint");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBack);

	rtt::RotateAroundPoint rotateAroundPoint(n);
	ros::Publisher pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
	// TODO: advertising on publisher is slow	
	ROS_INFO("waiting for connection");
	while(pub.getNumSubscribers() < 1){sleep(0.1);}
	ROS_INFO("connected");


	roboteam_utils::Vector2 rotaround=roboteam_utils::Vector2(0.5,0);
	roboteam_utils::Vector2 faceTowardsPos = roboteam_utils::Vector2(3.0, 4.5);
	rotateAroundPoint.updateArgs(pub,0,faceTowardsPos,3.0,rotaround,0.1);
	while (ros::ok()) {
		ros::spinOnce();
		if(rotateAroundPoint.Update() != bt::Node::Status::Running){break;} // break; breaks the code if only one message is sent
		
	}
	ROS_INFO("Skill completed");
	return 0;
}