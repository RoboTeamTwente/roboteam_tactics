#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class Sleep : public Skill {
public:
	Sleep(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        return params;
    }
    std::string node_name() { return "Sleep"; }
private:
	ros::Publisher pubSleep;

    time_point start;
};


} // rtt