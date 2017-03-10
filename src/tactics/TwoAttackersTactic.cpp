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

#define RTT_CURRENT_DEBUG_TAG TwoAttackersTactic

namespace rtt {

RTT_REGISTER_TACTIC(TwoAttackersTactic);

TwoAttackersTactic::TwoAttackersTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}



void TwoAttackersTactic::Initialize() {
    tokens.clear();

    // RTT_DEBUG("Initializing TwoAttackersTactic \n");
    // ROS_INFO("Initializing TwoAttackersTactic");
    
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

    Vector2 standFreePos;
    std::string ourSide;
    ros::param::get("our_side", ourSide);
    if (ourSide == "left") {
        standFreePos = Vector2(1.5, 1.5);
    } else if (ourSide == "right") {
        standFreePos = Vector2(-1.5, -1.5);
    } else {
        ROS_WARN("TwoAttackersTactic: something went wrong in getting param ourSide");
    }
    
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

        bb.SetString("GetBall_A_AimAt", "robot");
        bb.SetInt("GetBall_A_AimAtRobot", secondAttackerID);
        bb.SetBool("GetBall_A_AimAtRobotOurTeam", true);

        bb.SetString("AimAt_secondAttacker_At", "robot");
        bb.SetInt("AimAt_secondAttacker_AtRobot", secondAttackerID);

        bb.SetString("ParamCheck_canIShoot_signal", "readyToReceiveBall");
        bb.SetString("ParamCheck_canIShoot_mode", "eq");
        bb.SetString("ParamCheck_canIShoot_value", "ready");

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

        bb.SetString("ParamSet_default_signal", "readyToReceiveBall");
        bb.SetString("ParamSet_default_value", "nope");

        bb.SetInt("StandFree_A_theirID", firstAttackerID);
        bb.SetString("StandFree_A_whichTeam", "us");
        bb.SetDouble("StandFree_A_xGoal", standFreePos.x);
        bb.SetDouble("StandFree_A_yGoal", standFreePos.y);

        bb.SetString("ParamSet_readyToReceive_signal", "readyToReceiveBall");
        bb.SetString("ParamSet_readyToReceive_value", "ready");

        bb.SetDouble("ShootAtGoal_A_kickVel", 5.0);

        bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", true);

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

    isThisYourFirstTimeHere = true;
    start = rtt::now();
}

bt::Node::Status TwoAttackersTactic::Update() {

    // ROS_INFO_STREAM("TwoAttackersTactic time: " << time_difference_milliseconds(now(), lastUpdate).count());

    // if (time_difference_milliseconds(lastUpdate, now()).count() > 500) {
    //     ROS_INFO("TwoAttackersTactic Update too long ago");
    //     Initialize();
    // }
    

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

    lastUpdate = now();

    return bt::Node::Status::Running;
}

} // rtt
