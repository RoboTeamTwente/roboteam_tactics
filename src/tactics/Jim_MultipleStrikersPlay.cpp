#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

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

#define RTT_CURRENT_DEBUG_TAG Jim_MultipleStrikersPlay

namespace rtt {

RTT_REGISTER_TACTIC(Jim_MultipleStrikersPlay);

Jim_MultipleStrikersPlay::Jim_MultipleStrikersPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}



void Jim_MultipleStrikersPlay::Initialize() {
    tokens.clear();

    RTT_DEBUGLN_TEAM("Initializing Jim_MultipleStrikersPlay");    
    if (RobotDealer::get_available_robots().size() < 1) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }

    roboteam_msgs::World world = LastWorld::get();
    
    std::vector<int> robots = RobotDealer::get_available_robots();

    int numStrikers = std::min((int) RobotDealer::get_available_robots().size(), 2);
    // int numDirectStrikers = std::min((int) RobotDealer::get_available_robots().size(), 2);
    
    RTT_DEBUGLN("numStrikers: %i", numStrikers);


    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();


    // ===========================
    // Create the striker roles
    // ===========================
    std::vector<Vector2> strikersDefaultPositions;
    strikersDefaultPositions.push_back(Vector2(2.0, 1.0));
    strikersDefaultPositions.push_back(Vector2(2.0, -1.0));

    // std::vector<Vector2> strikersDefaultPositions;
    // strikersDefaultPositions.push_back(Vector2(3.7, 2.0));
    // strikersDefaultPositions.push_back(Vector2(3.7, -2.0));

    std::vector<int> strikerIDs = Jim_MultipleDefendersPlay::getClosestRobots(robots, strikersDefaultPositions, world);
    
    
    for (int i = 0; i < numStrikers; i++) {

        int strikerID = strikerIDs.at(i);
        // RTT_DEBUGLN("Initializing Striker %i", strikerID);
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
        bb.SetBool("ReceiveBall_A_shootAtGoal", false);

        // Create message
        rd.tree = "rtt_jim/StrikerRole";
        
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }

    // std::vector<int> directStrikerIDs = Jim_MultipleDefendersPlay::getClosestRobots(robots, directStrikersDefaultPositions, world);

    // for (int i = 0; i < numDirectStrikers; i++) {

    //     int strikerID = directStrikerIDs.at(i);
    //     // RTT_DEBUGLN("Initializing Striker %i", strikerID);
    //     delete_from_vector(robots, strikerID);
    //     claim_robot(strikerID);

    //     roboteam_msgs::RoleDirective rd;
    //     rd.robot_id = strikerID;
    //     bt::Blackboard bb;

    //     // Set the robot ID
    //     bb.SetInt("ROBOT_ID", strikerID);
    //     bb.SetInt("KEEPER_ID", 5);

    //     // bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", false);
    //     // bb.SetBool("ReceiveBall_A_computePoint", true);
    //     bb.SetDouble("ReceiveBall_A_computePointCloseToX", directStrikersDefaultPositions.at(i).x);
    //     bb.SetDouble("ReceiveBall_A_computePointCloseToY", directStrikersDefaultPositions.at(i).y);
    //     // bb.SetBool("ReceiveBall_A_setSignal", true);
    //     // bb.SetBool("ReceiveBall_A_shootAtGoal", true);

    //     // Create message
    //     rd.tree = "rtt_jim/DirectStrikerRole";
        
    //     rd.blackboard = bb.toMsg();

    //     // Add random token and save it for later
    //     boost::uuids::uuid token = unique_id::fromRandom();
    //     tokens.push_back(token);
    //     rd.token = unique_id::toMsg(token);

    //     // Send to rolenode
    //     pub.publish(rd);
    // }

    start = rtt::now();
}


bt::Node::Status Jim_MultipleStrikersPlay::Update() {

    // int successCount = 0;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            if (status == bt::Node::Status::Success) {
                RTT_DEBUGLN("Jim_MultipleStrikersPlay succeeded!");
                return Status::Success;
            }

            // if (status == bt::Node::Status::Failure) {
            //     RTT_DEBUGLN("Jim_MultipleStrikersPlay failed :(");
            //     return Status::Failure;
            // }
        }
    }

    return Status::Running;
}

} // rtt