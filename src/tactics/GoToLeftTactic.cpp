#include <memory>
#include <iostream>

#include "roboteam_tactics/tactics/GoToLeftTactic.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

GoToLeftTactic::GoToLeftTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

static int a = 0;

void GoToLeftTactic::Initialize() {
    std::cout << "Initializing GoToLeftTactic...\n";
    std::cout << "Sending around messages\n";
}

bt::Node::Status GoToLeftTactic::Update() {
    a++;
    if (a > 5) {
        a = 0;
        return bt::Node::Status::Success;
    }

    std::cout << "Updating GoToLeftTactic!\n";
    return bt::Node::Status::Running;
}

} // rtt
