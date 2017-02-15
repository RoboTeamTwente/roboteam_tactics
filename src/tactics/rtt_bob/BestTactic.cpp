#include "roboteam_tactics/tactics/rtt_bob/BestTactic.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_TACTIC_F(rtt_bob, BestTactic);

BestTactic::BestTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void BestTactic::Initialize() {
    return;
}

bt::Node::Status BestTactic::Update() {
    return Status::Failure;
}

} // rtt

