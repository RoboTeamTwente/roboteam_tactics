#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"

namespace rtt {

class Skill : public bt::Leaf {
    public:

    Skill(Aggregator& aggregator, bt::Blackboard::Ptr blackboard = nullptr)
            : aggregator{aggregator}
            , blackboard{blackboard} {}
    virtual ~Skill() {}
    
    Aggregator& aggregator;
    bt::Blackboard::Ptr blackboard;
} ;

class Role : public bt::BehaviorTree {
    public:

    Role (Aggregator& aggregator, int robotId, bt::Blackboard::Ptr blackboard = nullptr)
            : aggregator{aggregator}
            , robotId{robotId}
            , blackboard{blackboard} {}

    Aggregator& aggregator;
    int robotId;
    bt::Blackboard::Ptr blackboard;
} ;

class Tactic : public bt::Leaf {

} ;

class Strategy : public bt::BehaviorTree {

} ;

}
