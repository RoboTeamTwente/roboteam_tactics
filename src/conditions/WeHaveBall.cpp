#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/conditions/WeHaveBall.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(WeHaveBall);

WeHaveBall::WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<WeHaveBall>(name);
    std::string internal = name + "_internal_teamhasball";
    blackboard->SetBool("our_team", true);
    team = new TeamHasBall(internal, blackboard);
}

bt::Node::Status WeHaveBall::Update() {
    return team->Update();
}

}
