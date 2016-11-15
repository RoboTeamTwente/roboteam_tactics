#include <algorithm>
#include <memory>
#include <iostream>
#include <random>

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

    std::string pos = private_bb->GetString("pos");
    int robot_count = private_bb->GetDouble("robots");

    std::cout << "Initializing tactic!\n";
    std::cout << "Robot count: " << std::to_string(robot_count) << "\n";

    claim_role_nodes(robot_count);
    auto allRobots = RobotDealer::get_available_robots();
    for (int i = 0; i < robot_count; ++i) {
        int random_index = get_rand_int(allRobots.size());
        std::cout << "Claiming " << std::to_string(allRobots.at(random_index)) << "\n";
        claim_robot(allRobots.at(random_index));
        allRobots.erase(allRobots.begin() + random_index);
    }

    std::cout << "Claimed role nodes: " << get_claimed_role_nodes().size() << "\n";

    int mod = 1;
    std::string our_field_side = "left";
    ros::param::get("our_field_side", our_field_side);
    if (our_field_side == "left") {
        mod = -1;
    }

    int i = 0;
    auto workers = get_claimed_role_nodes();
    auto robots = get_claimed_robots();
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(robots.begin(), robots.end(), g);

    for (auto robot : robots) {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", robot);

        if (pos == "left") {
            bb.SetDouble("AvoidRobots_X_xGoal", mod * 3);
        } else if (pos == "right") {
            bb.SetDouble("AvoidRobots_X_xGoal", mod * -3);
        } else {
            bb.SetDouble("AvoidRobots_X_xGoal", mod * 0);
        }

        bb.SetDouble("AvoidRobots_X_yGoal", (i + 0.5) * 0.5 * mod);
        bb.SetBool("AvoidRobots_X_endPoint", true);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.node_id = workers.at(i);
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
