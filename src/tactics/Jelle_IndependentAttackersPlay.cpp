#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jelle_IndependentAttackersPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define RTT_CURRENT_DEBUG_TAG Jelle_IndependentAttackersPlay
#define ROS_LOG_NAME "plays.Jelle_IAP"

namespace rtt {

RTT_REGISTER_TACTIC(Jelle_IndependentAttackersPlay);

Jelle_IndependentAttackersPlay::Jelle_IndependentAttackersPlay(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard) {}


void Jelle_IndependentAttackersPlay::Initialize() {

    // Stringstream used to create a one-line description of all that happens below
    std::stringstream statusString;
    statusString << "Reset | ";

    tokens.clear();

    std::vector<int> robots = getAvailableRobots();

    if (robots.size() < 1) {
        ROS_WARN_NAMED(ROS_LOG_NAME, "Not enough robots, cannot initialize...");
        return;
    }

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);

    // Max 6 attackers
    int numAttackers = std::min((int) robots.size(), 6);
//    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Initializing Jelle_IndependentAttackersPlay numAttackers: " << numAttackers);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Print the robots that will be attackers
//    std::stringstream vectorStr;
//    std::copy(robots.begin(), robots.end(), std::ostream_iterator<int>(vectorStr, ", "));
//    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Attackers: [" << vectorStr.str().c_str() << "]");

    // priority of profiles: striker - midfielder - winger - midfielder - striker - winger
    int profiles[6] = {0, 2, 1, 2, 0, 1};

    for (size_t i = 0; i < (size_t) numAttackers; i++) {

        int attackerID = robots.at(i);
        claim_robot(attackerID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = attackerID;
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", attackerID);
        bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());
        if (i < 6) {
            bb.SetInt("Positioning_A_profile", profiles[i]);
        }

        // Create message
        rd.tree = "rtt_jelle/GeneralAttacker";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);

        statusString << attackerID << "=" << profiles[i] << " ";
    }

    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, statusString.str());

}

bt::Node::Status Jelle_IndependentAttackersPlay::Update() {

//    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Updating...");

//    std::vector<int> robotsAvailable = getAvailableRobots();
//
//    ROS_INFO_STREAM_COND_NAMED(0 < (int)robotsAvailable.size(), ROS_LOG_NAME, "Robots available!");
//
//     bool allFailed = true;
//
//     if (tokens.size() == 0) {
//         allFailed = false;
//     }
//
//     for (auto token : tokens) {
//         if (feedbacks.find(token) != feedbacks.end()) {
//             Status status = feedbacks.at(token);
//             if (status == bt::Node::Status::Success) {
//                 ROS_INFO_NAMED(ROS_LOG_NAME, "Strikers succeeded!");
//                 return Status::Success;
//             }
//
//             allFailed &= status == bt::Node::Status::Failure;
//         } else {
//             allFailed = false;
//         }
//     }
//
//     if (allFailed) {
//         ROS_INFO_NAMED(ROS_LOG_NAME, "Strikers failed!");
//         return Status::Failure;
//     }



    return Status::Running;
}

} // rtt
