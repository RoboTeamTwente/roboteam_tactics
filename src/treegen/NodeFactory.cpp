#include <exception>
#include <memory>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/treegen/NodeFactory.h"

namespace rtt {

using namespace factories;

template <>
std::shared_ptr<bt::Node> generate_rtt_node<void>(std::string className, std::string nodeName, bt::Blackboard::Ptr bb) {
    ROS_INFO_STREAM("Generating: \"" << className << "\", \"" << nodeName << "\", " << bb);
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
