#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/WeHaveBall.h"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

class Jim_IndependentAttackersPlay : public Tactic {
    public:
    Jim_IndependentAttackersPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    Status Update();
    ros::NodeHandle n;

    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;

    time_point lastUpdate;
    time_point finishTime;
    time_point lastTimeWeHadBall;

    WeHaveBall weHaveBall;
};

}