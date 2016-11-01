#pragma once

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class GoToRightTactic : public Tactic {
    public:
    GoToRightTactic(bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    ros::NodeHandle n;
    ros::Publisher directivePub = n.advertise<roboteam_msgs::RoleDirective>("/role_directive", 10);
} ;

}
