#include <memory>
#include <iostream>

#include "roboteam_tactics/tactics/GoToLeftTactic.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

GoToLeftTactic::GoToLeftTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

bt::Node::Status GoToLeftTactic::Update() {
    return bt::Node::Status::Running;
}

} // rtt
