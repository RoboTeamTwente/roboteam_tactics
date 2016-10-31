#include <memory>
#include <iostream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/GoToRightTactic.h"
#include "roboteam_tactics/utils.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

GoToRightTactic::GoToRightTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

void GoToRightTactic::Initialize() {
    std::cout << "Initializing GoToRight...\n";

    roboteam_msgs::RoleDirective resetDirective;
    directivePub.publish(resetDirective);

    auto workers = getNodesSubscribedTo("/role_directive");
    for (auto worker : workers) {
        roboteam_msgs::RoleDirective wd;
    }
}

static int a = 0;

bt::Node::Status GoToRightTactic::Update() {
    a++;

    if (a > 5) {
        a = 0;
        return bt::Node::Status::Success;
    }

    std::cout << "Updating GoToRightTactic!\n";
    return bt::Node::Status::Running;
}

} // rtt
