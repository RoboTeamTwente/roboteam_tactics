#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

class solo_defender_tactic : public Tactic {
    public:
    solo_defender_tactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    Status Update();
    ros::NodeHandle n;

    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;
} ;

}