#include "roboteam_tactics/tactics/BasicDefenseTactic.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

BasicDefenseTactic::BasicDefenseTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

bt::Node::Status BasicDefenseTactic::Update() {
    std::cout << "Picking BasicDefenseTactic!\n";
    // pickedTactic = std::make_shared<BasicDefenseTactic>();
    return bt::Node::Status::Success;
}


} // rtt

