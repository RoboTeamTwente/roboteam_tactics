#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class KickSkill : public Skill {
public:
	KickSkill();
	void Initialize(ros::NodeHandle nh, int robotIDInput);
	double cleanAngle(double angle);
	Status Update();
private:
	ros::NodeHandle n;
	ros::Publisher pubKickSkill;
	int robotID;
};


} // rtt