#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/SoloAttacker2Tactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG SoloAttacker2Tactic

namespace rtt {

RTT_REGISTER_TACTIC(SoloAttacker2Tactic);

SoloAttacker2Tactic::SoloAttacker2Tactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void SoloAttacker2Tactic::Initialize() {
    tokens.clear();

    // RTT_DEBUG("Initializing SoloAttacker2Tactic \n");
    
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
        bb.SetInt("KEEPER_ID", 5);

        bb.SetString("GetBall_A_AimAt", "robot");
        bb.SetInt("GetBall_A_AimAtRobot", 1);
        bb.SetBool("GetBall_A_AimAtRobotOurTeam", true);

        bb.SetString("AimAt_A_At", "robot");
        bb.SetInt("AimAt_A_AtRobot", 1.5);

        bb.SetDouble("Kick_A_kickVel", 1.5);
        
        bb.SetString("GetBall_B_AimAt", "theirgoal");

        bb.SetDouble("Kick_B_kickVel", 4.0);


        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = attackerID;
        wd.tree = "qualification/SoloAttacker2Role";
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

bt::Node::Status SoloAttacker2Tactic::Update() {
    bool treeSucceeded = false;
    bool treeFailed = false;
    bool treeInvalid = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            if (status == bt::Node::Status::Success) {
                treeSucceeded = true;
            }
            if (status == bt::Node::Status::Failure) {
                treeFailed = true;
            }
            if (status == bt::Node::Status::Invalid) {
                treeInvalid = true;
            }
        }
    }

    if (treeSucceeded) {
        return bt::Node::Status::Success;
    }
    if (treeFailed) {
        return bt::Node::Status::Failure;
    }
    if (treeInvalid) {
        return bt::Node::Status::Invalid;
    }

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 40) {
        RTT_DEBUGLN("Tactic failed because it took too long");
        return Status::Failure;
    }

    return bt::Node::Status::Running;
}

} // rtt
