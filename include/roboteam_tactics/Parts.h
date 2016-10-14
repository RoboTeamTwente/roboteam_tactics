#pragma once

#include "ros/ros.h"

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"

namespace rtt {

class Leaf : public bt::Leaf {
    public:
    Leaf(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr)
            : bt::Leaf(blackboard)
            , name{name}
            {}
    virtual ~Leaf() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    bt::Blackboard::Ptr private_bb = std::make_shared<bt::Blackboard>();
    const std::string name;

    // Proxies that prefix an id for lookups in the global table.
    void SetBool(std::string key, bool value) { blackboard->SetBool(getPrefixedId(key), value); }
    bool GetBool(std::string key) { return blackboard->GetBool(getPrefixedId(key)); }
    bool HasBool(std::string key) const { return blackboard->HasBool(getPrefixedId(key)); }

    void SetInt(std::string key, int value)  { blackboard->SetInt(getPrefixedId(key), value); }
    int GetInt(std::string key) { return blackboard->GetInt(getPrefixedId(key)); }
    bool HasInt(std::string key) const  { return blackboard->HasInt(getPrefixedId(key)); }

    void SetFloat(std::string key, float value)  { blackboard->SetFloat(getPrefixedId(key), value); }
    float GetFloat(std::string key) { return blackboard->GetFloat(getPrefixedId(key)); }
    bool HasFloat(std::string key) const  { return blackboard->HasFloat(getPrefixedId(key)); }

    void SetDouble(std::string key, double value)  { blackboard->SetDouble(getPrefixedId(key), value); }
    double GetDouble(std::string key) { return blackboard->GetDouble(getPrefixedId(key)); }
    bool HasDouble(std::string key) const  { return blackboard->HasDouble(getPrefixedId(key)); }

    void SetString(std::string key, std::string value)  { blackboard->SetString(getPrefixedId(key), value); }
    std::string GetString(std::string key) { return blackboard->GetString(getPrefixedId(key)); }
    bool HasString(std::string key) const  { return blackboard->HasString(getPrefixedId(key)); }
    
    std::string getPrefixedId(std::string id) const {
        if (name.empty()) {
            return id;
        }

        return name + "_" + id;
    }
} ;

class Skill : public Leaf {
public:

    Skill(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(name, blackboard)
            , n{n}
            {}
    virtual ~Skill() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    ros::NodeHandle n;
} ;

class Condition : public Leaf {
	public:
	Condition(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(name, blackboard)
            {}
    virtual ~Condition() {}

    virtual Status Update() {
        return Status::Invalid;
    }
} ;

class Role : public bt::BehaviorTree {
    public:

    Role (int robotId, bt::Blackboard::Ptr blackboard = nullptr) {
        SetSharedBlackboard(blackboard);
    }
    virtual ~Role() {}

    virtual Status Update() {
        return Status::Invalid;
    }
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
