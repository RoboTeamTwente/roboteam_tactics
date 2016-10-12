#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"

namespace rtt {

class Skill : public bt::Leaf {
    public:

    Skill(Aggregator& aggregator, bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(blackboard)
            , aggregator{aggregator}
            {}
    virtual ~Skill() {}
    
    Aggregator& aggregator;
    bt::Blackboard::Ptr private_blackboard = std::make_shared<bt::Blackboard>();
} ;

class Condition : public bt::Leaf {
	public:

	Condition(bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(blackboard)
            {}
    virtual ~Condition() {}
    
    bt::Blackboard::Ptr blackboard;
    bt::Blackboard::Ptr private_blackboard = std::make_shared<bt::Blackboard>();
} ;

class Role : public bt::BehaviorTree {
    public:

    Role (Aggregator& aggregator, int robotId, bt::Blackboard::Ptr blackboard = nullptr)
            : aggregator{aggregator}
            , robotId{robotId}
            {
        SetSharedBlackboard(blackboard);
    }
    virtual ~Role() {}

    Aggregator& aggregator;
    int robotId;
} ;

class Tactic : public bt::Leaf {
    public:
    Tactic(bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(blackboard)
            {}
    virtual ~Tactic() {}

    virtual void distribute_roles() {}
    virtual bt::Node::Status update_roles() {}
} ;

class Strategy : public bt::BehaviorTree {
    public:
    Strategy() {}
    virtual ~Strategy() {}

} ;

}
