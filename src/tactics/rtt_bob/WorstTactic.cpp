#include "roboteam_tactics/tactics/rtt_bob/WorstTactic.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

namespace rtt_bob {

RTT_REGISTER_TACTIC_F(rtt_bob, WorstTactic);

WorstTactic::WorstTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void WorstTactic::Initialize() {
    return;
}

bt::Node::Status WorstTactic::Update() {
    return Status::Failure;
}

} // rtt_bob

} // rtt


