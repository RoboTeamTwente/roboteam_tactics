#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/AttackerTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG AttackerTactic

namespace rtt {

AttackerTactic::AttackerTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void AttackerTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing\n");
    
    if (RobotDealer::get_available_robots().size() < 4) {
        RTT_DEBUG("Not enough robots, cannot initialize.\n");
        // TODO: Want to pass failure here as well!
        return;
    }

    roboteam_msgs::World world = rtt::LastWorld::get();

    // This tactic directs two robots
    int primaryAttacker;
    int secondaryAttacker;

    std::vector<int> robots = RobotDealer::get_available_robots();
    
    primaryAttacker = get_robot_closest_to_ball(robots);
    delete_from_vector(robots, primaryAttacker);
    
    secondaryAttacker = get_robot_closest_to_their_goal(robots);
    delete_from_vector(robots, secondaryAttacker);

    claim_robots({primaryAttacker, secondaryAttacker});

    RTT_DEBUG("primaryAttacker: %i, secondaryAttacker:%i\n", primaryAttacker, secondaryAttacker);

    // auto pub = get_roledirective_publisher();
    ros::NodeHandle n;
    auto pub = n.advertise<roboteam_msgs::RoleDirective>("role_directive", 100);

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        // Attacker 1
        bb.SetInt("ROBOT_ID", primaryAttacker);

        // Get the ball!
        bb.SetBool("GetBall_A_intercept", false);
        
        // If you can see the goal, aim towards it
        bb.SetBool("AimAt_A_setRosParam", false);
        bb.SetString("AimAt_A_At", "theirgoal");

        // Else, if you can see the other attacker, aim to him
        bb.SetBool("CanSeeRobot_A_our_team", true);
        bb.SetInt("CanSeeRobot_A_targetID", secondaryAttacker);
        bb.SetBool("AimAt_B_setRosParam", true);
        bb.SetString("AimAt_B_At", "robot");
        bb.SetInt("AimAt_B_AtRobot", secondaryAttacker);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = primaryAttacker;
        wd.tree = "CoolTree";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(wd);
    }

    {

        // Fill blackboard with relevant info
        bt::Blackboard bb;
        // Attacker 2
        bb.SetInt("ROBOT_ID", secondaryAttacker);

        // Make sure you stand free to receive the ball
        bb.SetInt("StandFree_A_theirID", primaryAttacker);
        bb.SetBool("StandFree_A_setRosParam", true);
        bb.SetString("StandFree_A_whichTeam", "us");
        bb.SetDouble("StandFree_A_distanceFromPoint", 0.3);

        // Receive the ball
        bb.SetBool("GetBall_A_intercept", true);
        bb.SetBool("GetBall_A_getBallAtCurrentPos", true);

        // Aim at goal
        bb.SetBool("AimAt_A_setRosParam", false);
        bb.SetString("AimAt_A_At", "theirgoal");

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = secondaryAttacker;
        wd.tree = "SuperCoolTree";
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

bt::Node::Status AttackerTactic::Update() {
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

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 8) {
        return Status::Failure;
    }

    return bt::Node::Status::Running;
}

} // rtt
