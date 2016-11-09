#include "roboteam_tactics/utils/NodeFactory.h"
#include "roboteam_tactics/utils/SkillFactory.h"
#include <exception>
#include <memory>

namespace rtt {

typedef enum { SKILL, ROLE, TACTIC, INVALID } RunType;

RunType determine_type(const std::string& name) {
    if (allskills_set.count(name) > 0) {
        return SKILL;
    }
    if (alltrees_set.count(name) > 0) {
        return ROLE;
    }
    if (alltactics_set.count(name) > 0) {
        return TACTIC;
    }
    return INVALID;
}

std::shared_ptr<Skill> generate_skill(ros::NodeHandle n, const std::string& className, const std::string& nodeName, bt::Blackboard::Ptr bb) {
    return std::dynamic_pointer_cast<Skill, Leaf>(make_skill<>(n, className, nodeName, bb));
}

std::shared_ptr<bt::BehaviorTree> generate_role(const std::string& name, ros::NodeHandle& n, bt::Blackboard bb) {
    return std::make_shared<bt::BehaviorTree>(make_tree(name, n, &bb));
}

std::shared_ptr<bt::Node> generate_node(ros::NodeHandle& n, std::string className, std::string nodeName, bt::Blackboard::Ptr bb) {
    RunType type = determine_type(className);
    switch (type) {
        case SKILL:
        return generate_skill(n, className, nodeName, bb);
        case ROLE:
        return generate_role(className, n, *bb);
        default:
        throw std::logic_error("Not yet implemented");
    }
}

}