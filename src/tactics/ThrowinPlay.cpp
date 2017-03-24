#include <algorithm>
#include <memory>
#include <iostream>
#include <random>

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/ThrowinPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG ThrowinPlay

namespace rtt {

RTT_REGISTER_TACTIC(ThrowinPlay);

ThrowinPlay::ThrowinPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void ThrowinPlay::Initialize() {
    tokens.clear();

    std::string pos = GetString("pos"); 
    int robot_count = GetDouble("robots");

    std::cout << "Initializing tactic!\n";
    std::cout << "Robot count: " << std::to_string(robot_count) << "\n";
    std::cout << "Side :" << pos << "\n";

    auto allRobots = RobotDealer::get_available_robots();
    claim_robot(allRobots.at(1));

    auto robots = get_claimed_robots();
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    for (auto robot_id : robots) {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", robot_id);

        roboteam_msgs::RoleDirective wd;
        wd.robot_id = robot_id;
        wd.tree = "rtt_ewoud/ThrowinTaker";
        wd.blackboard = bb.toMsg();

        // is this neccesairy?
        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(wd);
    }
}

bt::Node::Status ThrowinPlay::Update() {
    bool allSucceeded = true;
    bool oneFailed = false;
    bool oneInvalid = false;
    std::cout << "update new" << std::endl;
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
