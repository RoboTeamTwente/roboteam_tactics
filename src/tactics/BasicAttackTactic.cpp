#include <memory>
#include <iostream>

#include "roboteam_tactics/tactics/BasicAttackTactic.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

BasicAttackTactic::BasicAttackTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

bt::Node::Status BasicAttackTactic::Update() {
    std::cout << "Picking BasicAttackTactic!\n";
    pickedTactic = std::make_shared<BasicAttackTactic>();
    return bt::Node::Status::Success;
}

} // rtt
