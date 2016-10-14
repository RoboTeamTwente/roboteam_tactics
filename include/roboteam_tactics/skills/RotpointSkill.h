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
	
    RotpointSkill(bt::Blackboard::Ptr blackboard = nullptr);
    double cleanAngle(double angle);
    void stoprobot(ros::Publisher pub, int RobotID);
    roboteam_utils::Vector2 worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation);
    void updateArgs(ros::Publisher pub, int robotID, double targetAngle, double w, roboteam_utils::Vector2 center, double radius);
    Status Update();
private:
	uint32_t prevworldseq;
	bool firstworld=true;
	int robotID;
	double targetAngle;
	double rotw;
	ros::Publisher pub;
	roboteam_utils::Vector2 center;
   	double radius=radius;
	
} ;

} // rtt
