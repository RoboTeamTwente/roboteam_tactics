#include <memory>
#include <iostream>

#include "roboteam_tactics/tactics/GoToRightTactic.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

GoToRightTactic::GoToRightTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

bt::Node::Status GoToRightTactic::Update() {
    return bt::Node::Status::Running;
}

} // rtt
