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

    auto allRobots = getAvailableRobots();
    claim_robot(allRobots.at(0));
    claim_robot(allRobots.at(1));

    auto robots = get_claimed_robots();
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    int KEEPER_ID = RobotDealer::get_keeper();
    // create throwin taker role
    bt::Blackboard bbThrowinTaker;
    bbThrowinTaker.SetInt("ROBOT_ID", robots[0]);
    bbThrowinTaker.SetInt("KEEPER_ID", KEEPER_ID);
    ScopedBB(bbThrowinTaker, "CanPassSafely_A")
            .setInt("passToRobot", robots[1]);

    ScopedBB(bbThrowinTaker, "AimAt_A")
            .setInt("AtRobot", robots[1]);

    roboteam_msgs::RoleDirective wdThrowinTaker;
    wdThrowinTaker.robot_id = robots[0];
    wdThrowinTaker.tree = "rtt_ewoud/ThrowinTaker";
    wdThrowinTaker.blackboard = bbThrowinTaker.toMsg();

    // Add random token and save it for later
    boost::uuids::uuid token = unique_id::fromRandom();
    tokens.push_back(token);
    wdThrowinTaker.token = unique_id::toMsg(token);

    // Send to rolenode
    pub.publish(wdThrowinTaker);

    // create receiver role
    bt::Blackboard bbThrowinReceiver;
    bbThrowinReceiver.SetInt("KEEPER_ID", KEEPER_ID);
    bbThrowinReceiver.SetInt("ROBOT_ID", robots[1]);

    roboteam_msgs::RoleDirective wdThrowinReceiver;
    wdThrowinReceiver.robot_id = robots[1];
    wdThrowinReceiver.tree = "rtt_ewoud/ThrowinReceiver";
    wdThrowinReceiver.blackboard = bbThrowinReceiver.toMsg();

    // Add random token and save it for later
    token = unique_id::fromRandom();
    tokens.push_back(token);
    wdThrowinReceiver.token = unique_id::toMsg(token);

    // Send to rolenode
    pub.publish(wdThrowinReceiver);

}

bt::Node::Status ThrowinPlay::Update() {
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
