#include <memory>
#include <iostream>

#include "roboteam_msgs/Int32Entry.h"
#include "roboteam_msgs/Float64Entry.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/GoToRightTactic.h"
#include "roboteam_tactics/utils.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

GoToRightTactic::GoToRightTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

void GoToRightTactic::Initialize() {/*
    std::cout << "Initializing GoToRight...\n";

    // Empty directive is reset all!
    roboteam_msgs::RoleDirective resetDirective;
    directivePub.publish(resetDirective);

    auto workers = getNodesSubscribedTo("/role_directive");
    int i = -3;
    for (auto worker : workers) {
        roboteam_msgs::RoleDirective wd;
        wd.node_id = worker;
        wd.tree = "GoToPosTree";

        roboteam_msgs::Int32Entry ie;
        ie.name = "ROBOT_ID";
        ie.value = 0;
        wd.blackboard.ints.push_back(ie);
    
        roboteam_msgs::Float64Entry fe;
        fe.name = "xGoal";
        fe.value = 3;
        wd.blackboard.doubles.push_back(fe);

        fe.name = "yGoal";
        fe.value = (double) i;
        wd.blackboard.doubles.push_back(fe);

        directivePub.publish(wd);
    }*/
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
