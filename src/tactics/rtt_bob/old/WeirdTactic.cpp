#include "roboteam_tactics/tactics/rtt_bob/old/WeirdTactic.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_TACTIC_F(rtt_bob/old, WeirdTactic);

WeirdTactic::WeirdTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void WeirdTactic::Initialize() {
}

bt::Node::Status WeirdTactic::Update() {
}

} // rtt


