#include <memory>
#include <iostream>

#include "roboteam_tactics/tactics/BasicAttackTactic.h"

namespace rtt {

BasicAttackTactic::BasicAttackTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

bt::Node::Status BasicAttackTactic::Update() {
    std::cout << "Picking BasicAttackTactic!\n";
    // pickedTactic = std::make_shared<BasicAttackTactic>();
    return bt::Node::Status::Success;
}

} // rtt
