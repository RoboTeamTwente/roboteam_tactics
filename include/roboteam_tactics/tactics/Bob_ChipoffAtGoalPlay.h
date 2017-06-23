#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/Draw.h"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

class Bob_ChipoffAtGoalPlay : public Tactic {
    public:
    Bob_ChipoffAtGoalPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    Status Update();
    ros::NodeHandle n;

    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;

    bool failImmediately;
    bool isThisYourFirstTimeHere = true;
    time_point lastUpdate;
    time_point finishTime;

    roboteam_msgs::RoleDirective taker;
    roboteam_msgs::RoleDirective receiver;

    Draw drawer;

};

}
