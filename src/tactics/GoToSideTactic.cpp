#include <memory>
#include <iostream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/GoToSideTactic.h"
#include "roboteam_tactics/utils/utils.h"

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
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", i);

        if (left) {
            bb.SetDouble("AvoidRobots_X_xGoal", 3);
        } else {
            bb.SetDouble("AvoidRobots_X_xGoal", -3);
        }

        bb.SetDouble("AvoidRobots_X_yGoal", i * 0.5);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.node_id = worker;
        wd.tree = "GoToPosTree";
        wd.blackboard = bb.toMsg();

        // Send to rolenode
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
