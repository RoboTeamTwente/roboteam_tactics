#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>
#include <algorithm>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_MultipleStrikersPlay.h"
#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#define ROS_LOG_NAME "plays.Jim_MSP"

namespace rtt {

RTT_REGISTER_TACTIC(Jim_MultipleStrikersPlay);

Jim_MultipleStrikersPlay::Jim_MultipleStrikersPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard)
        , weHaveBall("", blackboard)
        {}



void Jim_MultipleStrikersPlay::Initialize() {
    tokens.clear();

    std::vector<int> robots = getAvailableRobots();

    // if (getAvailableRobots().size() < 1) {
    if (robots.size() < 1) {
        ROS_DEBUG_NAMED(ROS_LOG_NAME, "Not enough robots, cannot initialize...");
        // TODO: Want to pass failure here as well!
        return;
    }

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);
    
    int numStrikers = std::min((int) robots.size(), 2);    
    ROS_DEBUG_NAMED(ROS_LOG_NAME, "Initializing numStrikers: %i", numStrikers);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    double xPos = std::min(ballPos.x + 4.5, 2.2);

    std::vector<Vector2> strikersDefaultPositions;
    strikersDefaultPositions.push_back(Vector2(xPos, 1.5));
    strikersDefaultPositions.push_back(Vector2(xPos, -1.5));

    std::vector<int> strikerIDs = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, strikersDefaultPositions, world);    

    for (size_t i = 0; i < (size_t) numStrikers; i++) {

        int strikerID = strikerIDs.at(i);
        delete_from_vector(robots, strikerID);
        claim_robot(strikerID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = strikerID;
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", strikerID);
        bb.SetInt("KEEPER_ID", 5);

        // bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", false);
        bb.SetBool("ReceiveBall_A_computePoint", true);
        bb.SetDouble("ReceiveBall_A_computePointCloseToX", strikersDefaultPositions.at(i).x);
        bb.SetDouble("ReceiveBall_A_computePointCloseToY", strikersDefaultPositions.at(i).y);
        bb.SetBool("ReceiveBall_A_setSignal", true);
        bb.SetBool("ReceiveBall_A_shouldFail", true);
        bb.SetDouble("ReceiveBall_A_acceptableDeviation", 0.4);
        // bb.SetBool("ReceiveBall_A_shootAtGoal", false);

        // Create message
        rd.tree = "rtt_jim/DirectStrikerRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }

    lastTimeWeHadBall = now();
}


bt::Node::Status Jim_MultipleStrikersPlay::Update() {
    return Status::Running;

//
//    bool allFailed = true;
//
//    if (tokens.size() == 0) {
//        allFailed = false;
//    }
//
//    if ((std::chrono::steady_clock::now() - lastTimeWeHadBall) >= std::chrono::milliseconds(2000)) {
//        ROS_DEBUG_NAMED(ROS_LOG_NAME, "Last time we had ball was 2 seconds ago... Returning Failure!");
//        return Status::Failure;
//    }
//
//    for (auto token : tokens) {
//        if (feedbacks.find(token) != feedbacks.end()) {
//            Status status = feedbacks.at(token);
//            if (status == bt::Node::Status::Success) {
//                ROS_DEBUG_NAMED(ROS_LOG_NAME, "Strikers succeeded!");
//                return Status::Success;
//            }
//
//            allFailed &= status == bt::Node::Status::Failure;
//        } else {
//            allFailed = false;
//        }
//    }
//
//    if (allFailed) {
//        ROS_DEBUG_NAMED(ROS_LOG_NAME, "Strikers failed!");
//        return Status::Failure;
//    }
//
//    return Status::Running;
}

} // rtt
