#pragma once
#include <cstdint>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {



class RotateAroundPoint : public Skill {
public:
	
    RotateAroundPoint(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();   
    void stoprobot(int RobotID);
    std::string node_name() { return "RotateAroundPoint"; }

private:
	uint32_t prevworldseq;
	bool firstworld=true;
	int robotID;
	// double targetAngle;
	roboteam_utils::Vector2 faceTowardsPos;
	double rotw;
	ros::Publisher pub;
	roboteam_utils::Vector2 center;
   	double radius=radius;
   	
	
} ;

} // rtt
