#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_MultipleStrikersPlay.h"
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
    if (RobotDealer::get_available_robots().size() < 2) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = RobotDealer::get_available_robots();
    int keeperID = RobotDealer::get_keeper();
    
    int ballGetter = get_robot_closest_to_ball(robots);
    RTT_DEBUGLN("ballGetter ID: %i", ballGetter);
    delete_from_vector(robots, ballGetter);
    claim_robot(ballGetter);

    int numStrikers = std::min((int) RobotDealer::get_available_robots().size(), 2);
    RTT_DEBUGLN("numStrikers: %i", numStrikers);


    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();


    // Create the Keeper
    {
        RTT_DEBUGLN("Initializing Keeper %i", keeperID);
        delete_from_vector(robots, keeperID);
        RobotDealer::claim_robot(keeperID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = keeperID;
        // activeRobots.push_back(keeperID);
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", keeperID);
        bb.SetInt("KEEPER_ID", keeperID);

        // Create message
        rd.tree = "rtt_jim/DefenderRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }


    // Ball Defender: drives towards the ball to block as much as possible of the view of goal of the attackers
    // if (robots.size() >= 1) {
    //     int ballDefenderID = get_robot_closest_to_point(robots, world, Vector2(world.ball.pos));

    //     RTT_DEBUGLN("Initializing BallDefender %i", ballDefenderID);
    //     delete_from_vector(robots, ballDefenderID);
    //     RobotDealer::claim_robot(ballDefenderID);

    //     roboteam_msgs::RoleDirective rd;
    //     rd.robot_id = ballDefenderID;
    //     activeRobots.push_back(ballDefenderID);
    //     bt::Blackboard bb;

    //     // Set the robot ID
    //     bb.SetInt("ROBOT_ID", ballDefenderID);
    //     bb.SetInt("KEEPER_ID", keeperID);

    //     bb.SetDouble("DistanceXToY_A_distance", 2.0);
    //     bb.SetDouble("SimpleKeeper_A_distanceFromGoal", 1.35);

    //     // Create message
    //     rd.tree = "rtt_jim/AggDefRole";
    //     rd.blackboard = bb.toMsg();

    //     // Add random token and save it for later
    //     boost::uuids::uuid token = unique_id::fromRandom();
    //     tokens.push_back(token);
    //     rd.token = unique_id::toMsg(token);

    //     // Send to rolenode
    //     pub.publish(rd);
    // }


    // Create the GetBallRole
    {
        roboteam_msgs::RoleDirective rd;
        rd.robot_id = ballGetter;
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", ballGetter);
        bb.SetInt("KEEPER_ID", 5);

        bb.SetBool("GetBall_A_passToBestAttacker", true); 

        // Create message
        rd.tree = "rtt_jim/GetBallRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }


    // Create the striker roles
    std::vector<Vector2> strikersDefaultPositions;
    strikersDefaultPositions.push_back(Vector2(1.5, 1.5));
    strikersDefaultPositions.push_back(Vector2(1.5, -1.5));

    
    for (int i = 0; i < numStrikers; i++) {

        int strikerID = get_robot_closest_to_their_goal(robots);
        RTT_DEBUGLN("Initializing Striker %i", strikerID);
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

    start = rtt::now();
}

bt::Node::Status Jim_MultipleStrikersPlay::Update() {

    int successCount = 0;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            if (status == bt::Node::Status::Success) {
                successCount++;
            }
        }
    }

    if (successCount >= 2) {
        RTT_DEBUGLN("Jim_MultipleStrikersPlay succeeded!");
        return Status::Success;
    }

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 25) {
        return Status::Failure;
    }

    return Status::Running;
}

} // rtt