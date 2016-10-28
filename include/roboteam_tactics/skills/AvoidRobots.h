#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include <vector>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class AvoidRobots : public Skill {
public:
	AvoidRobots(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
private:
    ros::NodeHandle n;
	ros::Publisher pub;
	// roboteam_utils::Vector2 prevDirection;

} ;

} // rtt
