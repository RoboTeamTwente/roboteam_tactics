
#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Failer.h"


// failer is a skill because behavior3 classfy it as an action, all other actions are skills so the failer is a skill as well.
namespace rtt {

Failer::Failer(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) {
}


bt::Node::Status Failer::Update() {

    return Status::Failure;
}

} // rtt
