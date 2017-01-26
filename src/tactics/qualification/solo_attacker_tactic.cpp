#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/solo_attacker_tactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG solo_attacker_tactic

namespace rtt {

RTT_REGISTER_TACTIC(AttackerTactic);

solo_attacker_tactic::solo_attacker_tactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void solo_attacker_tactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing Solo Attacker Tactic \n");
    
    if (RobotDealer::get_available_robots().size() < 1) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = RobotDealer::get_available_robots();
    
    // int attackerID = 0;
    uint attackerID = get_robot_closest_to_ball(robots);
    delete_from_vector(robots, attackerID);
    claim_robots({attackerID});

    roboteam_msgs::World world = LastWorld::get();
    roboteam_utils::Vector2 ballPos(world.ball.pos);
    roboteam_utils::Vector2 theirGoalPos = LastWorld::get_their_goal_center();
    double targetAngle = (theirGoalPos - ballPos).angle();

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the Solo Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", attackerID);

        // Set the targetAngle to face the goal when we get the ball
        bb.SetDouble("GetBall_A_targetAngle", targetAngle);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = secondaryAttacker;
        wd.tree = "solo_attacker_role";
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

bt::Node::Status solo_attacker_tactic::Update() {
    bool treeSucceeded;
    bool treeFailed;
    bool treeInvalid;
    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            treeSucceeded &= status == bt::Node::Status::Success;
            treeFailed &= status == bt::Node::Status::Failure;
            treeInvalid &= status == bt::Node::Status::Invalid;
        }
    }

    if (treeSucceeded) return bt::Node::Status::Success;
    if (treeFailed) return bt::Node::Status::Failure;
    if (treeInvalid) return bt::Node::Status::Invalid;

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 12) {
        return Status::Failure;
    }

    return bt::Node::Status::Running;
}

} // rtt
