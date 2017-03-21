#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/TeamHasBall.h"

namespace rtt {

/**
 * \class TheyHaveBall
 * \brief See YAML
 */
/*
 * Descr: Checks whether the opponents have the ball
 * Params: null
 */
class TheyHaveBall : public Condition {
    public:
    TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard);
    ~TheyHaveBall() { delete team; }
    Status Update() override;
    
    std::string node_name() { return "TheyHaveBall"; }
    private:
    TeamHasBall* team;
};

}