#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include <map>
#include <type_traits>

namespace rtt {
    
enum class BBArgumentType { Int, Float, Double, Bool, String };

typedef std::map<std::string, BBArgumentType> VerificationMap;
    
template<typename L>
struct CanVerify {
    template<typename T, T> struct Check;
    template<typename T>
    static std::true_type test(Check<VerificationMap(*)(), &T::required_params>*);
    template<typename T>
    static std::false_type test(...);
    static constexpr bool can_it = decltype(test<L>(0))::value;
};    
    
class Leaf : public bt::Leaf {
    public:    
    template<typename Impl>
    static constexpr bool validate_blackboard(typename std::enable_if<CanVerify<Impl>::can_it, bt::Blackboard::Ptr>::type blackboard) {
        if (blackboard == nullptr) return false;
            VerificationMap required = Impl::required_params();
        for (auto pair : required) {
            bool valid = true; // need initialized value due to constexpr
            switch (pair.second) {
                case BBArgumentType::Int:
                valid = blackboard->HasInt(pair.first);
                break;
                case BBArgumentType::Float:
                valid = blackboard->HasFloat(pair.first);
                break;
                case BBArgumentType::Double:
                valid = blackboard->HasDouble(pair.first);
                break;
                case BBArgumentType::Bool:
                valid = blackboard->HasBool(pair.first);
                break;
               case BBArgumentType::String:
                valid = blackboard->HasString(pair.first);
                break;
                default:
                throw std::logic_error("Incomplete switch statement in rtt::Leaf::validate_blackboard.");
            }
            if (!valid) return false;
        }
        return true;
    }
    
    template<typename Impl>
    static constexpr bool validate_blackboard(typename std::enable_if<!CanVerify<Impl>::can_it, bt::Blackboard::Ptr>::type blackboard) {
        return blackboard != nullptr;
    }
    
};

class Skill : public bt::Leaf {
    public:

    Skill(Aggregator& aggregator, bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(blackboard)
            , aggregator{aggregator}
            {}
    virtual ~Skill() {}

    virtual Status Update() {
        return Status::Invalid;
    }
    
    Aggregator& aggregator;
    bt::Blackboard::Ptr private_blackboard = std::make_shared<bt::Blackboard>();
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
