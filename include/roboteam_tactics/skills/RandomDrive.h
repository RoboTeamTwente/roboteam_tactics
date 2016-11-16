#pragma once

#include "ros/ros.h"

#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class RandomDrive : public Skill {
public:
    RandomDrive(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
private:
    ros::Publisher debug_pub;
    // Id to add to the debug point, so that multiple of these nodes don't interfere.
    int debug_id;

    AvoidRobots avoidRobots;

    roboteam_utils::Vector2 goal;
    float goal_angle;
    bool pick_new_goal;
};

} // rtt
