#include <exception>
#include <memory>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/treegen/NodeFactory.h"

namespace rtt {

// typedef enum { SKILL, TREE, TACTIC, CONDITION, INVALID } RunType;

using namespace factories;

// RunType determine_type(const std::string& name) {
    // if (getRepo<Factory<Skill>>().count(name) > 0) {
        // return SKILL;
    // }
    // if (getRepo<Factory<bt::BehaviorTree>>().count(name) > 0) {
        // return TREE;
    // }
    // if (getRepo<Factory<Tactic>>().count(name) > 0) {
        // return TACTIC;
    // }
    // if (getRepo<Factory<Condition>>().count(name) > 0) {
        // return TACTIC;
    // }
    // return INVALID;
// }

// template<
    // class T
// >
// std::shared_ptr<T> generate_leaf(const std::string& className, const std::string& nodeName, bt::Blackboard::Ptr bb) {
    // // return std::dynamic_pointer_cast<Skill, Leaf>(make_skill<>(className, nodeName, bb));
    // return getRepo<Factory<T>>()[className](nodeName, bb);
// }

// std::shared_ptr<bt::BehaviorTree> generate_role(const std::string& name, bt::Blackboard bb) {
    // return std::make_shared<bt::BehaviorTree>(make_tree(name, &bb));
// }

// template <
    // class T
// >
// std::shared_ptr<bt::Node> generate_node(std::string className, std::string nodeName, bt::Blackboard::Ptr bb) {
    // auto& repo = getRepo<Factory<T>>();
    // if (repo.count(className) > 0) {
        // return repo[className](nodeName, bb);
    // } else {
        // return nullptr;
    // }
    // // RunType type = determine_type(className);
    // // switch (type) {
        // // case SKILL:
        // // return generate_skill(className, nodeName, bb);
        // // case ROLE:
        // // return generate_role(className, *bb);
        // // default:
        // // throw std::logic_error("Not yet implemented");
    // // }
// }

template <>
std::shared_ptr<bt::Node> generate_rtt_node<void>(std::string className, std::string nodeName, bt::Blackboard::Ptr bb) {
    {
        auto skillPtr = generate_rtt_node<Skill>(className, nodeName, bb);
        if (skillPtr) {
            return skillPtr;
        }
    }

    {
        auto tacticPtr = generate_rtt_node<Tactic>(className, nodeName, bb);
        if (tacticPtr) {
            return tacticPtr;
        }
    }

    {
        auto conditionPtr = generate_rtt_node<Condition>(className, nodeName, bb);
        if (conditionPtr) {
            return conditionPtr;
        }
    }

    {
        auto treePtr = generate_rtt_node<bt::BehaviorTree>(className, nodeName, bb);
        if (treePtr) {
            return treePtr;
        }
    }

    return nullptr;
}

}
