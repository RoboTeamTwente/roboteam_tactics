#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

class DemoTactic : public Tactic {
    public:
    DemoTactic(bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    ros::NodeHandle n;
    ros::Publisher directivePub = n.advertise<roboteam_msgs::RoleDirective>("role_directive", 10);

    private:
    std::vector<boost::uuids::uuid> tokens;

    time_point start;
} ;

}
