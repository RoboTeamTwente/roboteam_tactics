#pragma once
#include <cstdint>

#include "ros/ros.h"

#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {



class RotpointSkill : public Skill {
public:
	
    RotpointSkill(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    double cleanAngle(double angle);
    roboteam_utils::Vector2 worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation);
    void updateArgs(ros::Publisher pub, int robotID, double targetAngle, double w);
    Status Update();
private:
	uint32_t prevworldseq;
	bool firstworld=true;
	int robotID;
	double targetAngle;
	double rotw;
	ros::Publisher pub;
	
} ;

} // rtt
