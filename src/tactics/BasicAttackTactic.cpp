#include <memory>
#include <iostream>

#include "roboteam_tactics/tactics/BasicAttackTactic.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_TACTIC(BasicAttackTactic);

BasicAttackTactic::BasicAttackTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

bt::Node::Status BasicAttackTactic::Update() {
    std::cout << "Picking BasicAttackTactic!\n";
    return bt::Node::Status::Success;
}

} // rtt
