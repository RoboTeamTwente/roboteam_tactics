#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/Cone.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
#include <boost/optional.hpp>

namespace rtt {
    
class StandFree : public Skill {

public:
    StandFree(std::string name, bt::Blackboard::Ptr blackboard);  
	boost::optional<Cone> MakeCoverCone(std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots, roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 targetPos);
    Status Update();
private:
    ros::NodeHandle n;
	AvoidRobots avoidRobots;
	ros::Publisher debugPub;
	ros::Publisher debugPubPoint;
};
    
}
