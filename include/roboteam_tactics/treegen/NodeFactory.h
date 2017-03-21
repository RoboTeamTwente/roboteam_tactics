#include "ros/ros.h"

#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

using namespace factories;

/**
 * \brief Builds a node
 * \tparam T The optional concrete type of Node. If not set, the type is inferred from className.
 * \param className The name of the type of Node to be generated.
 * \param nodeName The name of the new node, as specified in a behavior tree
 * \param bb The blackboard which should be passed to the new node
 * \return A pointer to the new node if construction was successful, or nullptr otherwise
 */
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

/**
 * \brief Specialization of the above template which tries to determine the class of the node under construction.
 * \param className The name of the class of the Node to be generated
 * \param nodeName The name of the new node, as specified in a behavior tree
 * \param bb The blackboard which should be passed to the new node
 * \return A pointer to the new node if construction was successful, or nullptr otherwise
 */
template <>
std::shared_ptr<bt::Node> generate_rtt_node<void>(std::string className, std::string nodeName, bt::Blackboard::Ptr bb);

}
