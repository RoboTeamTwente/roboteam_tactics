#include <algorithm>

#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

namespace factories {

TreeRegisterer::TreeRegisterer(const std::string &name, TreeConstructor tc) {
    auto treeFactory = [=](std::string, bt::Blackboard::Ptr bb) {
        // Name (first parameter) is not used
        return std::make_shared<bt::BehaviorTree>(tc(bb.get()));
    };

    getRepo<Factory<bt::BehaviorTree>>()[name] = treeFactory;
}

bool isTactic(std::string tactic) {
    auto tactics = get_entry_names<Tactic>();
    auto it = std::find(tactics.begin(), tactics.end(), tactic);

    return it != tactics.end();
}

bool isSkill(std::string skill) {
    auto skills = get_entry_names<Skill>();
    auto it = std::find(skills.begin(), skills.end(), skill);

    return it != skills.end();
}

bool isCondition(std::string condition) {
    auto conditions = get_entry_names<Condition>();
    auto it = std::find(conditions.begin(), conditions.end(), condition);

    return it != conditions.end();
}

bool isTree(std::string tree) {
    auto trees = get_entry_names<bt::BehaviorTree>();
    auto it = std::find(trees.begin(), trees.end(), tree);

    return it != trees.end();
}

} // factories

} // rtt
