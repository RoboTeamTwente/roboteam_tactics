#include "ros/ros.h"

#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/Dribble.h"
#include "roboteam_msgs/World.h"
#include "roboteam_tactics/utils/utils.h"

bool success;

void msgCallBackDribble(const roboteam_msgs::WorldConstPtr& world, rtt::Dribble* dribble_B) {
	rtt::LastWorld::set(*world);
	if (dribble_B->Update() == bt::Node::Status::Success) {
		success = true;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "DribbleTest");
	ros::NodeHandle n;

	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetInt("ROBOT_ID", 0);
	bb->SetDouble("goalx", 1.0);
	bb->SetDouble("goaly", 0.0);
		
	
	rtt::Dribble dribble_B("", bb);
	
	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackDribble, _1, &dribble_B));
	int count=0;
	auto start=rtt::now();
	
	while (ros::ok()) {
		ros::spinOnce();
		if (success) {
			break;
		}
		
		auto nu=rtt::now();
		if(rtt::time_difference_seconds(start,nu).count()>5){
			bb->SetDouble("goaly",2.0);
		}
		
	}
	ROS_INFO("Dribble Test completed");

	return 0;
}
