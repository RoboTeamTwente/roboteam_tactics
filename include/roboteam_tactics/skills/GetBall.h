#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class GetBall : public Skill {
public:
    GetBall(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    void UpdateArgs(int robotIDInput);
	Status Update();
private:
	roboteam_utils::Vector2 prevTargetPos;
	double prevTargetAngle;
	roboteam_msgs::RobotCommand prevCommand;
	ros::NodeHandle n;
	ros::Publisher pubGetBall;
	GoToPos goToPos;
	int robotID;
} ;

} // rtt
