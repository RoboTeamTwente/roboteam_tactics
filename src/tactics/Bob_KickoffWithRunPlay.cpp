#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Bob_KickoffWithRunPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define RTT_CURRENT_DEBUG_TAG Bob_KickoffWithRunPlay

namespace rtt {

RTT_REGISTER_TACTIC(Bob_KickoffWithRunPlay);

Bob_KickoffWithRunPlay::Bob_KickoffWithRunPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}



void Bob_KickoffWithRunPlay::Initialize() {
    tokens.clear();
    failImmediately = false;

    RTT_DEBUGLN_TEAM("Initializing Bob_KickoffWithRunPlay");
    
    std::vector<int> robots = RobotDealer::get_available_robots();

    if (robots.size() < 2) {
        RTT_DEBUGLN("Less than two robots detected; cannot use play!");
        failImmediately = true;
    }

    int firstAttackerID = robots[0];
    int secondAttackerID = robots[1];
    int keeperID = RobotDealer::get_keeper();

    RobotDealer::claim_robots({firstAttackerID, secondAttackerID});

    firstAttacker.robot_id = firstAttackerID;
    secondAttacker.robot_id = secondAttackerID;

    Vector2 receivePos(4.5 / 2, 3 / -2.0);
    Vector2 thresholdPos(-1, 3 / -2.0);

    // std::string ourSide;
    // ros::param::get("our_side", ourSide);
    // if (ourSide == "left") {
        // standFreePos = Vector2(1.5, 1.5);
    // } else if (ourSide == "right") {
        // standFreePos = Vector2(-1.5, -1.5);
    // } else {
        // ROS_WARN("Bob_KickoffWithRunPlay: something went wrong in getting param ourSide");
        // standFreePos = Vector2(-1.5, -1.5);
    // }
    
    // delete_from_vector(robots, attackerID);
    // claim_robots({firstAttackerID, secondAttackerID});

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the first Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", firstAttackerID);
        bb.SetInt("KEEPER_ID", keeperID);

        ScopedBB(bb, "GetBallAndLookAtGoal")
            .setString("aimAt", "theirGoal")
            .setBool("passOn", false)
            .setString("stayOnSide", "ourSide")
            ;

        ScopedBB(bb, "ReceiverPassedThreshold")
            .setString("X", std::to_string(secondAttackerID))
            .setString("Y", "fixed point")
            .setDouble("px", thresholdPos.x)
            .setDouble("py", thresholdPos.y)
            .setString("mode", "lt")
            .setDouble("distance", 0.5)
            ;

        ScopedBB(bb, "GetBallAndShootAtReceiver");

        // bb.SetString("GetBall_A_AimAt", "robot");
        // bb.SetInt("GetBall_A_AimAtRobot", secondAttackerID);
        // bb.SetBool("GetBall_A_AimAtRobotOurTeam", true);

        // bb.SetString("AimAt_secondAttacker_At", "robot");
        // bb.SetInt("AimAt_secondAttacker_AtRobot", secondAttackerID);

        // bb.SetString("ParamCheck_canIShoot_signal", "readyToReceiveBall");
        // bb.SetString("ParamCheck_canIShoot_mode", "eq");
        // bb.SetString("ParamCheck_canIShoot_value", "ready");

        // Create message
        firstAttacker.tree = "rtt_bob/Kickoff_Taker";
        firstAttacker.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        firstAttacker.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(firstAttacker);
    }


    // Create the second Attacker Role
    // {
        // bt::Blackboard bb;

        // // Set the robot ID
        // bb.SetInt("ROBOT_ID", secondAttackerID);
        // bb.SetInt("KEEPER_ID", 5);

        // bb.SetString("ParamSet_default_signal", "readyToReceiveBall");
        // bb.SetString("ParamSet_default_value", "nope");

        // bb.SetInt("StandFree_A_theirID", firstAttackerID);
        // bb.SetString("StandFree_A_whichTeam", "us");
        // bb.SetDouble("StandFree_A_xGoal", standFreePos.x);
        // bb.SetDouble("StandFree_A_yGoal", standFreePos.y);
        // bb.SetBool("StandFree_A_seeGoal", true);

        // bb.SetString("ParamSet_readyToReceive_signal", "readyToReceiveBall");
        // bb.SetString("ParamSet_readyToReceive_value", "ready");

        // // bb.SetDouble("ShootAtGoal_A_kickVel", 5.0);
        
        // ScopedBB(bb, "AimAt_A")
            // .setString("At", "theirgoal")
            // ;

        // bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", true);

        // // Create message
        // secondAttacker.tree = "qualification/TwoAttackersSecondRole";
        // secondAttacker.blackboard = bb.toMsg();

        // // Add random token and save it for later
        // boost::uuids::uuid token = unique_id::fromRandom();
        // tokens.push_back(token);
        // secondAttacker.token = unique_id::toMsg(token);

        // // Send to rolenode
        // pub.publish(secondAttacker);
    // }

    isThisYourFirstTimeHere = true;
    start = rtt::now();
}

bt::Node::Status Bob_KickoffWithRunPlay::Update() {
    if (failImmediately) return Status::Failure;

    bool firstAttackerSucceeded = false;
    bool secondAttackerSucceeded = false;
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
