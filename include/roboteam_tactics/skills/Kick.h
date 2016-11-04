#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class Kick : public Skill {
public:
	Kick(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize(int robotIDInput);
	double cleanAngle(double angle);
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
private:
	ros::NodeHandle n;
	ros::Publisher pubKick;
	int robotID;
};


} // rtt
