#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/TeamHasBall.h"

namespace rtt {

class TheyHaveBall : public Condition {
    public:
    TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard);
    ~TheyHaveBall() { delete team; }
    Status Update() override;
    
    private:
    TeamHasBall* team;
};

}