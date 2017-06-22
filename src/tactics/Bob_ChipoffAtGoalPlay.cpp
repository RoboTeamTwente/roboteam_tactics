#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Bob_ChipoffAtGoalPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define RTT_CURRENT_DEBUG_TAG Bob_ChipoffAtGoalPlay

namespace rtt {

RTT_REGISTER_TACTIC(Bob_ChipoffAtGoalPlay);

Bob_ChipoffAtGoalPlay::Bob_ChipoffAtGoalPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void Bob_ChipoffAtGoalPlay::Initialize() {
    tokens.clear();
    failImmediately = false;

    RTT_DEBUGLN_TEAM("Initializing Bob_ChipoffAtGoalPlay");
    
    std::vector<int> robots = RobotDealer::get_available_robots();
    Vector2 receivePos(4.5 / 2, 3 / -2.0);
    Vector2 startPos(-3, 3 / -2.0);
    Vector2 thresholdPos(-1, 3 / -2.0);
    Vector2 rendezvous(3, -2);

    if (robots.size() < 2) {
        RTT_DEBUGLN("Less than two robots detected; cannot use play!");
        failImmediately = true;
        return;
    }

    // int takerID = robots[0];
    int takerID = get_robot_closest_to_ball(robots);
    robots.erase(std::remove(robots.begin(), robots.end(), takerID), robots.end());

    // int receiverID = robots[1];
    int receiverID = get_robot_closest_to_point(robots, LastWorld::get(), startPos);
    robots.erase(std::remove(robots.begin(), robots.end(), receiverID), robots.end());

    int keeperID = RobotDealer::get_keeper();

    claim_robots({takerID, receiverID});

    taker.robot_id = takerID;
    receiver.robot_id = receiverID;

    drawer.drawPoint("threshold!", thresholdPos);
    drawer.drawPoint("rendezvous!", rendezvous);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the first Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", takerID);
        bb.SetInt("KEEPER_ID", keeperID);

        ScopedBB(bb, "GetBallAndLookAtGoal")
            .setString("aimAt", "theirgoal")
            .setBool("passOn", false)
            .setString("stayOnSide", "ourSide")
            ;

        ScopedBB(bb, "ReceiverPassedThreshold")
            .setString("X", std::to_string(receiverID))
            .setString("Y", "fixed point")
            .setDouble("px", thresholdPos.x)
            .setDouble("py", thresholdPos.y)
            .setString("mode", "lt")
            .setDouble("distance", 0.5)
            ;

        ScopedBB(bb, "GetBallAndShootAtReceiver")
            .setDouble("targetAngle", (rendezvous - receivePos).angle())
            .setBool("ourTeam", true)
            .setBool("passOn", true)
            ;

        // bb.SetString("GetBall_A_AimAt", "robot");
        // bb.SetInt("GetBall_A_AimAtRobot", receiverID);
        // bb.SetBool("GetBall_A_AimAtRobotOurTeam", true);

        // bb.SetString("AimAt_receiver_At", "robot");
        // bb.SetInt("AimAt_receiver_AtRobot", receiverID);

        // bb.SetString("ParamCheck_canIShoot_signal", "readyToReceiveBall");
        // bb.SetString("ParamCheck_canIShoot_mode", "eq");
        // bb.SetString("ParamCheck_canIShoot_value", "ready");

        // Create message
        taker.tree = "rtt_bob/Kickoff_Taker";
        taker.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        taker.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(taker);
    }


    // Create the second Attacker Role
    {
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", receiverID);
        bb.SetInt("KEEPER_ID", keeperID);

        ScopedBB(bb, "GoToStartPos")
            .setDouble("xGoal", startPos.x)
            .setDouble("yGoal", startPos.y)
            .setDouble("angleGoal", (rendezvous - startPos).angle())
            .setBool("avoidRobots", true)
            ;

        ScopedBB(bb, "TakerHasBall")
            .setInt("me", takerID)
            .setBool("our_team", true)
            ;

        ScopedBB(bb, "ReceiveBallAtRendezVous")
            .setDouble("receiveBallAtX", rendezvous.x)
            .setDouble("receiveBallAtY", rendezvous.y)
            .setDouble("acceptableDeviation", 0.5)
            ;

        ScopedBB(bb, "GetBallAndShootAtGoal")
            .setString("aimAt", "theirgoal")
            .setBool("passOn", true)
            ;

        // Create message
        receiver.tree = "rtt_bob/Kickoff_Receiver";
        receiver.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        receiver.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(receiver);
    }

    isThisYourFirstTimeHere = true;
    start = rtt::now();
}

bt::Node::Status Bob_ChipoffAtGoalPlay::Update() {
    if (failImmediately) return Status::Failure;

    bool takerSucceeded = false;
    bool receiverSucceeded = false;
    bool oneFailed = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);

            if (status == bt::Node::Status::Success) {
                if (token == unique_id::fromMsg(taker.token)) {
                    // std::cout << "First attacker succeeded!\n";
                    takerSucceeded = true;
                }
                if (token == unique_id::fromMsg(receiver.token)) {
                    // std::cout << "Second attacker succeeded!\n";
                    receiverSucceeded = true;
                }
            } 

            if (status == bt::Node::Status::Failure) {
                std::cout << "One failed!\n";
                oneFailed = true;
            }
        }
    }

    if (takerSucceeded && receiverSucceeded) {
        RTT_DEBUGLN("Both roles succeeded, so tactic succeeded");
        return bt::Node::Status::Success;
    }

    if (oneFailed) {
        RTT_DEBUGLN("One role failed, so tactic failed");
        std::cout << "FAILING!!\n";
        return bt::Node::Status::Failure;
    }

    // auto duration = time_difference_seconds(start, now());
    // if (duration.count() >= 25) {
        // return Status::Failure;
    // }

    lastUpdate = now();

    return bt::Node::Status::Running;
}

} // rtt
