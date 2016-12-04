#include "ros/ros.h"

#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

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
}

template <>
std::shared_ptr<bt::Node> generate_rtt_node<void>(std::string className, std::string nodeName, bt::Blackboard::Ptr bb);

}
