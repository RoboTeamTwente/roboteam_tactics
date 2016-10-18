#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include <vector>

#include "roboteam_msgs/navsim.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class FollowPath : public Skill {
public:
    FollowPath(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    std::vector<roboteam_msgs::Point> ComputePath(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 goalPos);
    void CallGoToPos(roboteam_msgs::Point point, double wGoal, int robotID);
	Status Update();
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::ServiceClient client;
	std::vector<roboteam_msgs::Point> points;

	GoToPos* goToPos;
	int robotID;
	int state; // 1: compute path, 2: go to a position along the path, 3: check if done, or move to next position
} ;

} // rtt
