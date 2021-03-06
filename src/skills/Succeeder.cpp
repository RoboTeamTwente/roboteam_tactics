#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Succeeder.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG Succeeder

// failer is a skill because behavior3 classfy it as an action, all other actions are skills so the failer is a skill as well.
namespace rtt {

RTT_REGISTER_SKILL(Succeeder);

Succeeder::Succeeder(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) {
}


bt::Node::Status Succeeder::Update() {
    return Status::Success;
}

} // rtt
