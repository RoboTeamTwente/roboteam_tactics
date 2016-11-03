#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/conditions/WeHaveBall.h"
#include "roboteam_tactics/utils/LastWorld.h"

namespace rtt {
    
WeHaveBall::WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<WeHaveBall>(name);
    std::string internal = name + "_internal_teamhasball";
    blackboard->SetBool("blue_team", LastWorld::we_are_blue());
    team = new TeamHasBall(internal, blackboard);
}

bt::Node::Status WeHaveBall::Update() {
    return team->Update();
}
    
}
