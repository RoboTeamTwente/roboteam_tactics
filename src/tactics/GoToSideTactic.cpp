#include <algorithm>
#include <memory>
#include <iostream>
#include <random>

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/GoToSideTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_TACTIC(GoToSideTactic);

GoToSideTactic::GoToSideTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void GoToSideTactic::Initialize() {
    tokens.clear();

    std::string pos = GetString("pos"); 
    int robot_count = GetDouble("robots");

    std::cout << "Initializing tactic!\n";
    std::cout << "Robot count: " << std::to_string(robot_count) << "\n";
    std::cout << "Side :" << pos << "\n";

    auto allRobots = getAvailableRobots();
    for (int i = 0; i < robot_count; ++i) {
        int random_index = get_rand_int(allRobots.size());
        std::cout << "Claiming " << std::to_string(allRobots.at(random_index)) << "\n";
        claim_robot(allRobots.at(random_index));
        allRobots.erase(allRobots.begin() + random_index);
    }

    int mod = 1;
    std::string our_side = get_our_side();
    if (our_side == "left") {
        mod = -1;
    }

    int i = 0;
    auto robots = get_claimed_robots();
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(robots.begin(), robots.end(), g);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

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
        wd.robot_id = robot;
        wd.tree = "GoToPosTree";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(wd);
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
