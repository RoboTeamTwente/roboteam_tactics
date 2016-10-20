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

bool success;

void msgCallBackRotateAroundPoint(const roboteam_msgs::WorldConstPtr& world, rtt::RotateAroundPoint* rotateAroundPoint, bt::Blackboard::Ptr bb) {
	rtt::LastWorld::set(*world);
	roboteam_msgs::World getworld = rtt::LastWorld::get();
	roboteam_msgs::WorldBall ball = getworld.ball;
	roboteam_utils::Vector2 center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	roboteam_utils::Vector2 faceTowardsPos = roboteam_utils::Vector2(3.0, 0.0);
	bb->SetDouble("faceTowardsPosx", faceTowardsPos.x);
    bb->SetDouble("faceTowardsPosy", faceTowardsPos.y);
	bb->SetDouble("centerx", center.x);
    bb->SetDouble("centery", center.y);
    double radius = 0.09;
    bb->SetDouble("radius", radius);
	if (rotateAroundPoint->Update() == bt::Node::Status::Success) {
		success = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "RotateAroundPoint");
	ros::NodeHandle n;
	// ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBack);
	
	ros::Publisher pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
	// TODO: advertising on publisher is slow	
	ROS_INFO("waiting for connection");
	while(pub.getNumSubscribers() < 1){sleep(0.1);}
	ROS_INFO("connected");

	roboteam_utils::Vector2 faceTowardsPos = roboteam_utils::Vector2(3.0, 0.0);
	
	roboteam_msgs::World world = rtt::LastWorld::get();
	roboteam_msgs::WorldBall ball = world.ball;

	// roboteam_msgs::WorldRobot robot = world.us.at(0);

	roboteam_utils::Vector2 center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
	int radius = 0.09;

	auto bb = std::make_shared<bt::Blackboard>();
    bb->SetDouble("faceTowardsPosx", faceTowardsPos.x);
    bb->SetDouble("faceTowardsPosy", faceTowardsPos.y);
    bb->SetDouble("centerx", center.x);
    bb->SetDouble("centery", center.y);
    bb->SetDouble("radius", radius);
    bb->SetDouble("w",3.0);
    bb->SetInt("ROBOT_ID", 0);
    

	rtt::RotateAroundPoint rotateAroundPoint(n, "", bb);
	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackRotateAroundPoint, _1, &rotateAroundPoint, bb));
	
	while (ros::ok()) {

		// roboteam_msgs::World world = rtt::LastWorld::get();
		// roboteam_msgs::WorldBall ball = world.ball;
		// roboteam_msgs::WorldRobot robot = world.us[0];
		// roboteam_utils::Vector2 center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
		// bb->SetDouble("centerx", center.x);
	 //    bb->SetDouble("centery", center.y);
	 //    bb->SetDouble("radius", radius);

		ros::spinOnce();
		if (success) {
			break;
		}
	}
	ROS_INFO("Skill completed");
	return 0;
}
