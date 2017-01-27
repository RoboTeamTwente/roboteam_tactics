#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/SoloAttackerTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG SoloAttackerTactic

namespace rtt {

RTT_REGISTER_TACTIC(SoloAttackerTactic);

SoloAttackerTactic::SoloAttackerTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void SoloAttackerTactic::Initialize() {
    tokens.clear();

    // RTT_DEBUG("Initializing SoloAttackerTactic \n");
    
    if (RobotDealer::get_available_robots().size() < 1) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = RobotDealer::get_available_robots();
    
    int attackerID = 0;
    // delete_from_vector(robots, attackerID);
    claim_robots({attackerID});

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the Solo Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", attackerID);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = attackerID;
        wd.tree = "qualification/SoloAttackerRole";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(wd);
    }

    start = rtt::now();
}

bt::Node::Status SoloAttackerTactic::Update() {
    bool treeSucceeded = false;
    bool treeFailed = false;
    bool treeInvalid = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            treeSucceeded &= status == bt::Node::Status::Success;
            treeFailed &= status == bt::Node::Status::Failure;
            treeInvalid &= status == bt::Node::Status::Invalid;
        }
    }

    if (treeSucceeded) {
        // RTT_DEBUGLN("SoloAttackerTactic Succeeded!");
        return bt::Node::Status::Success;
    }
    if (treeFailed) {
        // RTT_DEBUGLN("SoloAttackerTactic Failed!");
        return bt::Node::Status::Failure;
    }
    if (treeInvalid) {
        // RTT_DEBUGLN("SoloAttackerTactic Invalid!");
        return bt::Node::Status::Invalid;
    }

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 12) {
        return Status::Failure;
    }

    return bt::Node::Status::Running;
}

} // rtt
