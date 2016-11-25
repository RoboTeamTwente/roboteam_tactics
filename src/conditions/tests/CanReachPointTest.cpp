#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_tactics/skills/AvoidRobots.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

#include <ctime>
#include <chrono>

bool success;
bool failure;
bool finished1;

void msgCallBack(const roboteam_msgs::WorldConstPtr& world, rtt::CanReachPoint* canReachPoint) {
	rtt::LastWorld::set(*world);
	bt::Node::Status status = canReachPoint->Update();
	if (status == bt::Node::Status::Success) {
		success = true;
	}
	if (status == bt::Node::Status::Failure) {
		failure = true;
	}
}

void msgCallBackAvoidRobots(const roboteam_msgs::WorldConstPtr& world, rtt::AvoidRobots* avoidRobots1) {
	rtt::LastWorld::set(*world);
	if (!finished1) {
		if (avoidRobots1->Update() != bt::Node::Status::Running) {
			finished1 = true;
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "CanReachPointTest");
	ros::NodeHandle n;

	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetInt("ROBOT_ID", 1);
	bb->SetString("whichTeam", "us");
	bb->SetDouble("xGoal", 0.0);
	bb->SetDouble("yGoal", 0.0);
	bb->SetDouble("angleGoal", 3.14);
	bb->SetDouble("timeLimit", 1.0);
	bb->SetBool("shouldStop", true);

	rtt::CanReachPoint canReachPoint("", bb);
	{
		ros::Subscriber sub1 = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBack, _1, &canReachPoint));

		while (ros::ok()) {
			ros::spinOnce();
			if (success) {
				ROS_INFO("Yep, you can reach this point");
				break;
			}
			if (failure) {
				ROS_INFO("Nope, you can't reach this point :(");
				break;
			}
		}
	}


	auto bb1 = std::make_shared<bt::Blackboard>();
	bb1->SetInt("ROBOT_ID", 1);
	bb1->SetDouble("xGoal", 0.0);
	bb1->SetDouble("yGoal", 0.0);
	bb1->SetDouble("angleGoal", 3.14);
	// bb1->SetBool("priority", false);
	rtt::AvoidRobots avoidRobots1(n, "", bb1);

	ros::Subscriber sub2 = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBackAvoidRobots, _1, &avoidRobots1));

	time_t startTime;
	time(&startTime);

	auto start = std::chrono::steady_clock::now();

	while (ros::ok()) {
		ros::spinOnce();
		if (finished1) {
			// time(&finishTime);

			break;
		}
	}
	auto duration = std::chrono::duration_cast< std::chrono::milliseconds> 
                        (std::chrono::steady_clock::now() - start);

	double timeDiff = duration.count()/1000.0;
	ROS_INFO_STREAM("time to reach target: " << timeDiff);

	ROS_INFO("Test completed!");

	return 0;
}
