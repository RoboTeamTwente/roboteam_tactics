#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/TeamHasBall.h"

namespace rtt {

class WeHaveBall : public Condition {
    public:
    WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard);
    ~WeHaveBall() { delete team; }
    Status Update() override;
    
    std::string node_name() { return "WeHaveBall"; }
    private:
    TeamHasBall* team;
};

}