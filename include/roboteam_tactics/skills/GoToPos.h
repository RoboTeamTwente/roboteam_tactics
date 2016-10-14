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

class GoToPos : public Skill {
public:
    GoToPos();
    void Initialize(ros::NodeHandle nh, int robotIDInput);
    void UpdateArgs(double xGoalInput, double yGoalInput, double wGoalInput, bool endPoint);
	Status Update();
private:
	roboteam_msgs::World prevWorld;
    ros::NodeHandle n;
	ros::Publisher pub;
	double xGoal;
	double yGoal;
	double wGoal;
	bool endPoint;
	int robotID;
	
} ;

} // rtt