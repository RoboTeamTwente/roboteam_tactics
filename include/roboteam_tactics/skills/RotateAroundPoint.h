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



class RotateAroundPoint : public Skill {
public:
	
    RotateAroundPoint(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();   
    double cleanAngle(double angle);
    void stoprobot(int RobotID);
    roboteam_utils::Vector2 worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation);
    double computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos);

   
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
