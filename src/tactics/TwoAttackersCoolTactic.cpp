#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/TwoAttackersCoolTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG TwoAttackersCoolTactic

namespace rtt {

RTT_REGISTER_TACTIC(TwoAttackersCoolTactic);

TwoAttackersCoolTactic::TwoAttackersCoolTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void TwoAttackersCoolTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing TwoAttackersCoolTactic \n");
    
    if (RobotDealer::get_available_robots().size() < 1) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = RobotDealer::get_available_robots();

    int firstAttackerID = 0;
    int secondAttackerID = 1;
    firstAttacker.robot_id = firstAttackerID;
    secondAttacker.robot_id = secondAttackerID;
    
    // delete_from_vector(robots, attackerID);
    claim_robots({firstAttackerID, secondAttackerID});

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the first Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", firstAttackerID);
        bb.SetInt("KEEPER_ID", 5);

        bb.SetDouble("GoToPos_A_xGoal", -1.0);
        bb.SetDouble("GoToPos_A_yGoal", 0.0);
        bb.SetDouble("GoToPos_A_angleGoal", M_PI);
        bb.SetBool("GoToPos_A_avoidRobots", true);

        bb.SetString("AimAt_A_At", "position");
        bb.SetDouble("AimAt_A_xGoal", -4.5);
        bb.SetDouble("AimAt_A_yGoal", 3.0);

        bb.SetDouble("Kick_A_kickVel", 3.0);

        // Create message
        firstAttacker.tree = "qualification/TwoAttackersCoolFirstRole";
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

        bb.SetDouble("GoToPos_A_xGoal", -1.0);
        bb.SetDouble("GoToPos_A_yGoal", 1.0);
        bb.SetDouble("GoToPos_A_angleGoal", M_PI);
        bb.SetBool("GoToPos_A_avoidRobots", true);

        bb.SetString("GetBall_A_AimAt", "theirgoal");
        bb.SetBool("GetBall_A_passOn", true);
        
        bb.SetDouble("Kick_A_kickVel", 5.0);
        

        // Create message
        secondAttacker.tree = "qualification/TwoAttackersCoolSecondRole";
        secondAttacker.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        secondAttacker.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(secondAttacker);
    }

    isThisYourFirstTimeHere = true;
    start = rtt::now();
}

bt::Node::Status TwoAttackersCoolTactic::Update() {

    bool firstAttackerSucceeded = false;
    bool secondAttackerSucceeded = false;
    bool oneFailed = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);

            if (status == bt::Node::Status::Success) {
                if (token == unique_id::fromMsg(firstAttacker.token)) {
                    firstAttackerSucceeded = true;
                    firstAttacker.tree = firstAttacker.STOP_EXECUTING_TREE;
                    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
                    pub.publish(firstAttacker);
                }
                if (token == unique_id::fromMsg(secondAttacker.token)) {
                    secondAttackerSucceeded = true;
                    secondAttacker.tree = secondAttacker.STOP_EXECUTING_TREE;
                    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
                    pub.publish(secondAttacker);
                }
            } 
            if (status == bt::Node::Status::Failure) {
                oneFailed = true;
            }
        }
    }

    if (firstAttackerSucceeded && secondAttackerSucceeded) {
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

    return bt::Node::Status::Running;
}

} // rtt
