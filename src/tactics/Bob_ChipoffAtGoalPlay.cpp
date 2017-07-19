#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Bob_ChipoffAtGoalPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define RTT_CURRENT_DEBUG_TAG Bob_ChipoffAtGoalPlay

namespace rtt {

RTT_REGISTER_TACTIC(Bob_ChipoffAtGoalPlay);

Bob_ChipoffAtGoalPlay::Bob_ChipoffAtGoalPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void Bob_ChipoffAtGoalPlay::Initialize() {
    tokens.clear();
    failImmediately = false;

    std::cout << "ChipoffAtGoalPlay!\n";

    RTT_DEBUGLN_TEAM("Initializing Bob_ChipoffAtGoalPlay");
    
    std::vector<int> robots = getAvailableRobots();

    if (robots.size() < 1) {
        RTT_DEBUGLN("Less than one robot detected; cannot use play!");
        failImmediately = true;
        return;
    }

    auto takerID = get_robot_closest_to_ball(robots);
    if (!takerID) {
    	failImmediately = true;
    }
    robots.erase(std::remove(robots.begin(), robots.end(), takerID), robots.end());

    int keeperID = RobotDealer::get_keeper();

    claim_robots({*takerID});

    taker.robot_id = *takerID;

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the first Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", *takerID);
        bb.SetInt("KEEPER_ID", keeperID);

        ScopedBB(bb, "GetBall_")
            .setString("aimAt", "theirgoal")
            .setBool("passOn", true)
            .setString("stayOnSide", "ourSide")
            ;

        // Create message
        taker.tree = "rtt_bob/ChipAtGoal";
        taker.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        taker.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(taker);
    }

    isThisYourFirstTimeHere = true;
    start = rtt::now();
}

bt::Node::Status Bob_ChipoffAtGoalPlay::Update() {
    if (failImmediately) return Status::Failure;

    bool takerSucceeded = false;
   // bool receiverSucceeded = false;
    bool oneFailed = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);

            if (status == bt::Node::Status::Success) {
                if (token == unique_id::fromMsg(taker.token)) {
                    // std::cout << "First attacker succeeded!\n";
                    takerSucceeded = true;
                }
                if (token == unique_id::fromMsg(receiver.token)) {
                    // std::cout << "Second attacker succeeded!\n";
                    //receiverSucceeded = true;
                }
            } 

            if (status == bt::Node::Status::Failure) {
                std::cout << "One failed!\n";
                oneFailed = true;
            }
        }
    }

    if (takerSucceeded) {
        return bt::Node::Status::Success;
    }

    if (oneFailed) {
        RTT_DEBUGLN("One role failed, so tactic failed");
        return bt::Node::Status::Failure;
    }

    lastUpdate = now();

    return bt::Node::Status::Running;
}

} // rtt
