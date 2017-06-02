#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

class Jim_MultipleStrikersPlay : public Tactic {
    public:
    Jim_MultipleStrikersPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    Status Update();
    ros::NodeHandle n;

    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;

    time_point lastUpdate;
    time_point finishTime;
};

}