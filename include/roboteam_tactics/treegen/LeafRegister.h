#pragma once

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

using TreeConstructor = std::function<bt::BehaviorTree(ros::NodeHandle, bt::Blackboard)>;
using TreeFactory = std::function<std::shared_ptr<bt::BehaviorTree>(bt::Blackboard*)>;
template <
    class T
>
using Factory = std::function<std::shared_ptr<T>(std::string name, bt::Blackboard::Ptr)>;

Repo<TreeFactory>& getTreeFactoryMap();
Repo<Factory<Condition>>& getConditionFactoryMap();
Repo<Factory<Skill>>& getSkillFactoryMap();
Repo<Factory<Tactic>>& getTacticFactoryMap();

class TreeRegisterer {
    TreeRegisterer(const std::string &name, TreeConstructor tc) {
        auto treeFactory = [=](bt::Blackboard* bb) {
            ros::NodeHandle n;
            return std::make_shared<bt::BehaviorTree>(tc(n, *bb));
        };
        getTreeFactoryMap()[name] = treeFactory;
    }
} ;

template <
    class L
>
class LeafRegisterer {
    LeafRegisterer(const std::string &name, std::function<Repo<Factory<L>>&()> factoryRepoGetter) {
        auto leafFactory = [=](std::string name, bt::Blackboard::Ptr bb) {
            return std::make_shared<L>(name, bb);
        };

        factoryRepoGetter()[name] = leafFactory;
    }
} ;

}  // factories

} // rtt
