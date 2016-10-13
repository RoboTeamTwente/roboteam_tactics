#pragma once

#include "ros/ros.h"

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"

namespace rtt {

class Skill : public bt::Leaf {
    public:

    Skill(ros::NodeHandle n, bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(blackboard)
            , n{n}
            {}
    virtual ~Skill() {}

    virtual Status Update() {
        return Status::Invalid;
    }
    
    bt::Blackboard::Ptr private_blackboard = std::make_shared<bt::Blackboard>();
    ros::NodeHandle n;
} ;

class Condition : public bt::Leaf {
	public:

	Condition(bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(blackboard)
            {}
    virtual ~Condition() {}

    virtual Status Update() {
        return Status::Invalid;
    }
    
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

    virtual Status Update() {
        return Status::Invalid;
    }

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
    virtual bt::Node::Status update_roles() {
        return bt::Node::Status::Invalid;
    }
} ;

class Strategy : public bt::BehaviorTree {
    public:
    Strategy() {}
    virtual ~Strategy() {}

    virtual Status Update() {
        return Status::Invalid;
    }
} ;

}
