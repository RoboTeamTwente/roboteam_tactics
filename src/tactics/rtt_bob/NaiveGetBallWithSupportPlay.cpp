#include "roboteam_tactics/tactics/rtt_bob/NaiveGetBallWithSupportPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

namespace rtt_bob {

RTT_REGISTER_TACTIC_F(rtt_bob, NaiveGetBallWithSupportPlay);

NaiveGetBallWithSupportPlay::NaiveGetBallWithSupportPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void NaiveGetBallWithSupportPlay::Initialize() {
    return;
}

bt::Node::Status NaiveGetBallWithSupportPlay::Update() {
    return Status::Running;
}

}

} // rtt

