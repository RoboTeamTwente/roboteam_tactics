#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_tactics/Parts.h"

namespace rtt {
    
class Qualification1v1Tactic : public Tactic {
    public:
    Qualification1v1Tactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize() override;
    Status Update() override;
    
    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;
    roboteam_msgs::RoleDirective firstRD, secondRD;
    bool canRun;
};
    
}