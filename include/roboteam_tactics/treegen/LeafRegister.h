#pragma once

#include <iostream>
#include <string>
#include <map>
#include <functional>
#include <memory>

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

namespace factories {

template <
    class T
>
using Repo = std::map<std::string, T>;

using TreeConstructor = std::function<bt::BehaviorTree(bt::Blackboard*)>;
template <
    class T
>
using Factory = std::function<std::shared_ptr<T>(std::string name, bt::Blackboard::Ptr)>;

template <
    class T
>
Repo<T>& getRepo() {
    static Repo<T> repo;
    return repo;
}

class TreeRegisterer {
public:
    TreeRegisterer(const std::string &name, TreeConstructor tc);
} ;

template <
    class L,
    class Parent
>
class LeafRegisterer {
public:
    LeafRegisterer(const std::string &name) {
        auto leafFactory = [=](std::string name, bt::Blackboard::Ptr bb) {
            return std::make_shared<L>(name, bb);
        };

        getRepo<Factory<Parent>>()[name] = leafFactory;
    }
} ;

} // factories

} // rtt

// Macros (anonymous namespace s.t. it doesn't leak outside the source file
// Double colon is for startin resolution from global namespace
#define RTT_REGISTER_LEAF(leafName, typeName) \
    namespace { \
    ::rtt::factories::LeafRegisterer<leafName, typeName> leafName ## _registerer(#leafName); \
    }

#define RTT_REGISTER_SKILL(skillName) RTT_REGISTER_LEAF(skillName, Skill)
#define RTT_REGISTER_CONDITION(conditionName) RTT_REGISTER_LEAF(conditionName, Condition)
#define RTT_REGISTER_TACTIC(tacticName) RTT_REGISTER_LEAF(tacticName, Tactic)
