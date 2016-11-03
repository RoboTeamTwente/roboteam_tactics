#include "roboteam_tactics/conditions/TheyHaveBall.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {
    
TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<TheyHaveBall>(name);
    std::string internal = name + "_internal_teamhasball";
    blackboard->SetBool("blue_team", !LastWorld::we_are_blue());
    team = new TeamHasBall(internal, blackboard);
}

bt::Node::Status TheyHaveBall::Update() {
    return team->Update();
}
    
}
