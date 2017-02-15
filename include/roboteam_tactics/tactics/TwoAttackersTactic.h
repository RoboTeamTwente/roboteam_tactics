#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

class TwoAttackersTactic : public Tactic {
    public:
    TwoAttackersTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    Status Update();
    ros::NodeHandle n;

    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;

    bool isThisYourFirstTimeHere = true;
    time_point finishTime;

    roboteam_msgs::RoleDirective firstAttacker;
    roboteam_msgs::RoleDirective secondAttacker;
};

}
