#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/TwoAttackersTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define RTT_CURRENT_DEBUG_TAG TwoAttackersTactic

namespace rtt {

RTT_REGISTER_TACTIC(TwoAttackersTactic);

TwoAttackersTactic::TwoAttackersTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}



void TwoAttackersTactic::Initialize() {
    tokens.clear();

    RTT_DEBUGLN_TEAM("Initializing TwoAttackersTactic");    
    // if (getAvailableRobots().size() < 1) {
    //     RTT_DEBUG("Not enough robots, cannot initialize... \n");
    //     // TODO: Want to pass failure here as well!
    //     return;
    // }
    
    std::vector<int> robots = getAvailableRobots();

    firstAttackerID = 0;
    secondAttackerID = 1;
    thirdAttackerID = 2;
    firstAttacker.robot_id = firstAttackerID;
    secondAttacker.robot_id = secondAttackerID;
    thirdAttacker.robot_id = thirdAttackerID;

    // delete_from_vector(robots, attackerID);
    // claim_robots({firstAttackerID, secondAttackerID});

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the first Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", firstAttackerID);
        bb.SetInt("KEEPER_ID", 5);

        // bb.SetString("GetBall_A_aimAt", "robot");
        bb.SetBool("GetBall_A_passToBestAttacker", true); 
        // bb.SetInt("GetBall_A_aimAtRobot", secondAttackerID);

        // bb.SetString("AimAt_A_At", "robot");
        // bb.SetInt("AimAt_A_AtRobot", secondAttackerID);

        bb.SetBool("Kick_A_wait_for_signal", true);

        // Create message
        firstAttacker.tree = "qualification/TwoAttackersFirstRole";
        firstAttacker.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        firstAttacker.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(firstAttacker);
    }


    // Create the second Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", secondAttackerID);
        bb.SetInt("KEEPER_ID", 5);

        // bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", false);
        bb.SetBool("ReceiveBall_A_computePoint", true);
        bb.SetBool("ReceiveBall_A_setSignal", true);

        // Create message
        secondAttacker.tree = "qualification/TwoAttackersSecondRole";
        secondAttacker.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        secondAttacker.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(secondAttacker);
    }

    // Create the third Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", thirdAttackerID);
        bb.SetInt("KEEPER_ID", 5);

        // bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", false);
        bb.SetBool("ReceiveBall_A_computePoint", true);
        bb.SetBool("ReceiveBall_A_setSignal", true);

        // Create message
        thirdAttacker.tree = "qualification/TwoAttackersSecondRole";
        thirdAttacker.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        thirdAttacker.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(thirdAttacker);
    }

    isThisYourFirstTimeHere = true;
    start = rtt::now();
}

bt::Node::Status TwoAttackersTactic::Update() {
    bool firstAttackerSucceeded = false;
    bool secondAttackerSucceeded = false;
    bool thirdAttackerSucceeded = false;
    bool oneFailed = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);

            if (status == bt::Node::Status::Success) {
                if (token == unique_id::fromMsg(firstAttacker.token)) {
                    firstAttackerSucceeded = true;
                }
                if (token == unique_id::fromMsg(secondAttacker.token)) {
                    secondAttackerSucceeded = true;
                }
                if (token == unique_id::fromMsg(thirdAttacker.token)) {
                    thirdAttackerSucceeded = true;
                }
            } 
            if (status == bt::Node::Status::Failure) {
                oneFailed = true;
            }
        }
    }

    if (firstAttackerSucceeded && secondAttackerSucceeded && thirdAttackerSucceeded) {
        RTT_DEBUGLN("Both roles succeeded, so tactic succeeded");
        return bt::Node::Status::Success;
    }

    if (oneFailed) {
        RTT_DEBUGLN("One role failed, so tactic failed");
        return bt::Node::Status::Failure;
    }

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 25) {
        return Status::Failure;
    }

    lastUpdate = now();

    return bt::Node::Status::Running;
}

} // rtt
