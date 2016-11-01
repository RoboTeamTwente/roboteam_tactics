#include <memory>
#include <iostream>

#include "roboteam_msgs/Int32Entry.h"
#include "roboteam_msgs/Float64Entry.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/GoToSideTactic.h"
#include "roboteam_tactics/utils.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

GoToSideTactic::GoToSideTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

void GoToSideTactic::Initialize() {
    bool left = private_bb->GetString("left") == "true";

    if (left) {
        std::cout << "Going left...\n";
    } else {
        std::cout << "Going right...\n";
    }

    // Empty directive is reset all!
    roboteam_msgs::RoleDirective resetDirective;
    directivePub.publish(resetDirective);

    auto workers = getNodesSubscribedTo("/role_directive");
    int i = 0;
    for (auto worker : workers) {
        roboteam_msgs::RoleDirective wd;
        wd.node_id = worker;
        wd.tree = "GoToPosTree";

        roboteam_msgs::Int32Entry ie;
        ie.name = "ROBOT_ID";
        ie.value = i;
        wd.blackboard.ints.push_back(ie);
    
        roboteam_msgs::Float64Entry fe;
        fe.name = "AvoidRobots_X_xGoal";

        if (left) fe.value = 3;
        else fe.value = -3;

        wd.blackboard.doubles.push_back(fe);

        fe.name = "AvoidRobots_X_yGoal";
        fe.value = (double) i * 0.5;
        wd.blackboard.doubles.push_back(fe);

        directivePub.publish(wd);
        i++;
    }
}

static int a = 0;

bt::Node::Status GoToSideTactic::Update() {
    a++;

    if (a > 600) {
        a = 0;
        return bt::Node::Status::Success;
    }

    // std::cout << "Updating GoToSideTactic!\n";
    return bt::Node::Status::Running;
}

} // rtt
