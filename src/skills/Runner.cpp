#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Runner.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG Runner

// runner is a skill because behavior3 classfy it as an action, all other actions are skills so the runner is a skill as well.
namespace rtt {

RTT_REGISTER_SKILL(Runner);

Runner::Runner(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) {
}


bt::Node::Status Runner::Update() {
	RTT_DEBUG("Inside runner!\n");
    return Status::Running;
}

} // rtt
