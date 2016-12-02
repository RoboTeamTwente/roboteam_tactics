#include "ros/ros.h"
// #include "roboteam_tactics/Parts.h"
// #include "roboteam_tactics/generated/allskills_factory.h"
// #include "roboteam_tactics/generated/alltrees_factory.h"
//#include "roboteam_tactics/generated/alltactics_factory.h"
// #include "roboteam_tactics/generated/allskills_set.h"
// #include "roboteam_tactics/generated/alltrees_set.h"
// #include "roboteam_tactics/generated/alltactics_set.h"

#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

// std::shared_ptr<bt::Node> generate_node(std::string className, std::string nodeName, bt::Blackboard::Ptr bb);

using namespace factories;

template <
    class T = void
>
std::shared_ptr<bt::Node> generate_rtt_node(std::string className, std::string nodeName, bt::Blackboard::Ptr bb) {
    auto& repo = getRepo<Factory<T>>();
    if (repo.count(className) > 0) {
        return repo[className](nodeName, bb);
    } else {
        return nullptr;
    }
    // RunType type = determine_type(className);
    // switch (type) {
        // case SKILL:
        // return generate_skill(className, nodeName, bb);
        // case ROLE:
        // return generate_role(className, *bb);
        // default:
        // throw std::logic_error("Not yet implemented");
    // }
}

template <>
std::shared_ptr<bt::Node> generate_rtt_node<void>(std::string className, std::string nodeName, bt::Blackboard::Ptr bb);

}
