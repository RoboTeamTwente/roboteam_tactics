#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class Kick : public Skill {
public:
	Kick(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize(int robotIDInput);
	double cleanAngle(double angle);
	Status Update();
private:
	ros::NodeHandle n;
	ros::Publisher pubKick;
	int robotID;
};


} // rtt
