#include "roboteam_tactics/treegen/LeafRegister.h"

rtt::factories::TreeRegisterer::TreeRegisterer(const std::string &name, TreeConstructor tc) {
    auto treeFactory = [=](std::string, bt::Blackboard::Ptr bb) {
        // Name (first parameter) is not used
        return std::make_shared<bt::BehaviorTree>(tc(bb.get()));
    };

    getRepo<Factory<bt::BehaviorTree>>()[name] = treeFactory;
}
