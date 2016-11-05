#include <memory>
#include <iostream>

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/GoToSideTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"

namespace rtt {

GoToSideTactic::GoToSideTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

void GoToSideTactic::Initialize() {
    tokens.clear();

    bool left = private_bb->GetString("left") == "true";

    if (left) {
        std::cout << "Going left...\n";
    } else {
        std::cout << "Going right...\n";
    }

    // Empty directive is reset all!
    roboteam_msgs::RoleDirective resetDirective;
    directivePub.publish(resetDirective);

    auto workers = getNodesSubscribedTo(getMyNamespace() + "role_directive");
    std::cout << "Number of workers: " << std::to_string(workers.size()) << "\n";
    std::cout << "My namespace: " << getMyNamespace() << "\n";
    int i = 0;

    int mod = 1;
    std::string our_field_side = "left";
    ros::param::get("our_field_side", our_field_side);
    if (our_field_side == "left") {
        mod = -1;
    }

    for (auto worker : workers) {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", i);



        if (left) {
            bb.SetDouble("AvoidRobots_X_xGoal", mod * 3);
        } else {
            bb.SetDouble("AvoidRobots_X_xGoal", mod * -3);
        }

        bb.SetDouble("AvoidRobots_X_yGoal", i * 0.5);
        bb.SetBool("AvoidRobots_X_endPoint", true);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.node_id = worker;
        wd.tree = "GoToPosTree";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        directivePub.publish(wd);
        i++;
    }
}

static int a = 0;

bt::Node::Status GoToSideTactic::Update() {
    bool allSucceeded = true;
    bool oneFailed = false;
    bool oneInvalid = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            allSucceeded &= status == bt::Node::Status::Success;
            oneFailed |= status == bt::Node::Status::Failure;
            oneInvalid |= status == bt::Node::Status::Invalid;
        } else {
            allSucceeded = false;
        }
    }

    if (oneFailed) {
        return bt::Node::Status::Failure;
    } else if (oneInvalid) {
        return bt::Node::Status::Invalid;
    } else if (allSucceeded) {
        return bt::Node::Status::Success;
    }

    return bt::Node::Status::Running;
}

} // rtt
