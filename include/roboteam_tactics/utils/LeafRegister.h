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

}  // factories

} // rtt
