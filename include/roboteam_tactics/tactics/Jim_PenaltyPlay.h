#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

class Jim_PenaltyPlay : public Tactic {
    public:
    Jim_PenaltyPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    void ReleaseAllBots();
    Status Update();
    ros::NodeHandle n;

    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;
    time_point successTime;
    bool success;
    int activeRobot;
};

}