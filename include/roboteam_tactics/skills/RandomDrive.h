#pragma once

#include "ros/ros.h"

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class RandomDrive : public Skill {
public:
    RandomDrive(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }

    std::string node_name() { return "RandomDrive"; }
private:
    ros::NodeHandle n;
    ros::Publisher debug_pub;
    // Id to add to the debug point, so that multiple of these nodes don't interfere.
    int debug_id;

    GoToPos goToPos;

    roboteam_utils::Vector2 goal;
    float goal_angle;
    bool pick_new_goal;
};

} // rtt
