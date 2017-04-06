#include "roboteam_tactics/tactics/rtt_bob/NaiveGetBallWithSupportPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_TACTIC_F(rtt_bob, NaiveGetBallWithSupport);

NaiveGetBallWithSupport::NaiveGetBallWithSupport(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void NaiveGetBallWithSupport::Initialize() {
    return;
}

bt::Node::Status NaiveGetBallWithSupport::Update() {
    return Status::Running;
}

} // rtt

