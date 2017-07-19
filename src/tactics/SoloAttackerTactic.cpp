#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <ros/ros.h>


#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/SoloAttackerTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
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
    
    if (getAvailableRobots().size() < 1) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = getAvailableRobots();
    
    int attackerID = 7;
    int keeperID = 3;
    // delete_from_vector(robots, attackerID);
    claim_robots({attackerID, keeperID});

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the Solo Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", attackerID);
        bb.SetInt("KEEPER_ID", keeperID);

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

    // Vector2 theirGoalPos = LastWorld::get_our_goal_center();
    // Vector2 keeperPos(theirGoalPos.x - 0.3*signum(theirGoalPos.x), theirGoalPos.y);

    // {
    //     bt::Blackboard bb;

    //     // Set the robot ID
    //     bb.SetInt("ROBOT_ID", keeperID);
    //     bb.SetInt("KEEPER_ID", 5);

    //     bb.SetDouble("ReceiveBall_A_receiveBallAtX", keeperPos.x);
    //     bb.SetDouble("ReceiveBall_A_receiveBallAtY", keeperPos.y);
    //     bb.SetDouble("ReceiveBall_A_acceptableDeviation", 0.45);

    //     bb.SetString("AimAt_fieldCenter_AimAt", "position");
    //     bb.SetDouble("AimAt_fieldCenter_xGoal", 0.0);
    //     bb.SetDouble("AimAt_fieldCenter_yGoal", 0.0);

    //     bb.SetDouble("Kick_A_kickVel", 2.5);

    //     // Create message
    //     roboteam_msgs::RoleDirective wd;
    //     wd.robot_id = keeperID;
    //     wd.tree = "qualification/SoloDefenderRole";
    //     wd.blackboard = bb.toMsg();

    //     // Add random token and save it for later
    //     boost::uuids::uuid token = unique_id::fromRandom();
    //     tokens.push_back(token);
    //     wd.token = unique_id::toMsg(token);

    //     // Send to rolenode
    //     pub.publish(wd);
    // }

    start = rtt::now();
}

bt::Node::Status SoloAttackerTactic::Update() {
    // ROS_INFO("tactic update");
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
    if (duration.count() >= 60) {
        RTT_DEBUGLN("Tactic failed because it took too long");
        return Status::Failure;
    }

    return bt::Node::Status::Running;
}

} // rtt
