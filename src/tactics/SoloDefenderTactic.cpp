#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/SoloDefenderTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/Math.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG SoloDefenderTactic

namespace rtt {

RTT_REGISTER_TACTIC(SoloDefenderTactic);

SoloDefenderTactic::SoloDefenderTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void SoloDefenderTactic::Initialize() {
    tokens.clear();

    // RTT_DEBUG("Initializing Solo Defender Tactic \n");
    
    if (RobotDealer::get_available_robots().size() < 1) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = RobotDealer::get_available_robots();

    roboteam_utils::Vector2 theirGoalPos = LastWorld::get_our_goal_center();
    roboteam_utils::Vector2 keeperPos(theirGoalPos.x - 0.3*signum(theirGoalPos.x), theirGoalPos.y);
    
    int defenderID = 0;
    claim_robots({defenderID});

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the Solo Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", defenderID);

        bb.SetBool("GetBall_A_intercept", false);
        bb.SetBool("GetBall_A_isKeeper", true);
        bb.SetString("GetBall_A_AimAt", "fieldcenter");

        bb.SetDouble("GetBall_B_getBallAtX", keeperPos.x);
        bb.SetDouble("GetBall_B_getBallAtY", keeperPos.y);
        bb.SetDouble("GetBall_B_getBallAtTime", 12.0);
        bb.SetBool("GetBall_B_intercept", true);
        bb.SetDouble("GetBall_B_acceptableDeviation", 0.45);
        bb.SetBool("GetBall_B_isKeeper", true);

        bb.SetDouble("Kick_A_kickVel", 2.5);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = defenderID;
        wd.tree = "qualification/SoloDefenderRole";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(wd);
    }
}

bt::Node::Status SoloDefenderTactic::Update() {
    // bool treeSucceeded = false;
    // bool treeFailed = false;
    // bool treeInvalid = false;

    // for (auto token : tokens) {
    //     if (feedbacks.find(token) != feedbacks.end()) {
    //         Status status = feedbacks.at(token);
    //         treeSucceeded &= status == bt::Node::Status::Success;
    //         treeFailed &= status == bt::Node::Status::Failure;
    //         treeInvalid &= status == bt::Node::Status::Invalid;
    //     }
    // }

    // if (treeSucceeded) return bt::Node::Status::Success;
    // if (treeFailed) return bt::Node::Status::Failure;
    // if (treeInvalid) return bt::Node::Status::Invalid;

    // auto duration = time_difference_seconds(start, now());
    // if (duration.count() >= 12) {
    //     return Status::Failure;
    // }

    return bt::Node::Status::Running;
}

} // rtt
