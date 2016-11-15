#include "roboteam_tactics/tactics/BasicDefenseTactic.h"

namespace rtt {

BasicDefenseTactic::BasicDefenseTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

bt::Node::Status BasicDefenseTactic::Update() {
    std::cout << "Picking BasicDefenseTactic!\n";
    // pickedTactic = std::make_shared<BasicDefenseTactic>();
    return bt::Node::Status::Success;
}


} // rtt

