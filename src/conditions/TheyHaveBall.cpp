#include "roboteam_tactics/conditions/TheyHaveBall.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(TheyHaveBall);

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<TheyHaveBall>(name);
    std::string internal = name + "_internal_teamhasball";
    blackboard->SetBool("our_team", false);
    team = new TeamHasBall(internal, blackboard);
}

bt::Node::Status TheyHaveBall::Update() {
    return team->Update();
}

}
