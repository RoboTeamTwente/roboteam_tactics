#include "roboteam_tactics/tactics/BasicDefenseTactic.h"

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

