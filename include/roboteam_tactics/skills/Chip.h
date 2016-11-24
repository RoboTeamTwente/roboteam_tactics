#pragma once

#include "ros/ros.h"

#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"


namespace rtt {

class Chip : public Skill {
public:
    Chip(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize() override;
    Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }

    std::string node_name() { return "Chip"; }
private:
    ros::NodeHandle n;
    ros::Publisher pubChip;
    int robotID;

    roboteam_utils::Vector2 oldBallVel;
    int cycleCounter;
};

} // rtt
