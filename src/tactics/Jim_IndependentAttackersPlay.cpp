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
#include "roboteam_tactics/tactics/Jim_IndependentAttackersPlay.h"
#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_tactics/utils/RobotPatternGenerator.h"


#define RTT_CURRENT_DEBUG_TAG Jim_IndependentAttackersPlay
#define ROS_LOG_NAME "plays.Jim_IAP"

namespace rtt {

RTT_REGISTER_TACTIC(Jim_IndependentAttackersPlay);

Jim_IndependentAttackersPlay::Jim_IndependentAttackersPlay(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard), weHaveBall("", blackboard) {}



void Jim_IndependentAttackersPlay::Initialize() {
	ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Initializing...");

    tokens.clear();

	// Release any previously claimed robots
	RobotDealer::release_robots(robotsClaimed);
	robotsClaimed.clear();

	reInitializeWhenNeeded();
	return;


    std::vector<int> robots = getAvailableRobots();

    if (robots.size() < 1) {
        ROS_WARN_NAMED(ROS_LOG_NAME, "Not enough robots, cannot initialize...");
        return;
    }

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);

    // Max 3 attackers
    int numAttackers = std::min((int) robots.size(), 5);
    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Initializing Jim_IndependentAttackersPlay numAttackers: " << numAttackers);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    std::vector<Vector2> attackersDefaultPositions;
    attackersDefaultPositions.push_back(Vector2(3.0, 1.5));
    attackersDefaultPositions.push_back(Vector2(3.0, -1.5));
    attackersDefaultPositions.push_back(Vector2(1.0, 0.0));
    attackersDefaultPositions.push_back(Vector2(4.5, 2.0));
    attackersDefaultPositions.push_back(Vector2(4.5, -2.0));

    std::vector<int> strikerIDs = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, attackersDefaultPositions, world);

    // Print the robots that should be defended and intercepted
    std::stringstream vectorStr;
    std::copy(strikerIDs.begin(), strikerIDs.end(), std::ostream_iterator<int>(vectorStr, ", "));
    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Strikers : [" << vectorStr.str().c_str() << "]");

    numAttackers = std::min((int) strikerIDs.size(), numAttackers);    

    for (size_t i = 0; i < (size_t) numAttackers; i++) {

        int strikerID = strikerIDs.at(i);
        delete_from_vector(robots, strikerID);
        claim_robot(strikerID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = strikerID;
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", strikerID);
        bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());

        // bb.SetBool("ReceiveBall_B_computePoint", true);
        bb.SetDouble("ReceiveBall_A_acceptableDeviation", 1.5);
        bb.SetDouble("ReceiveBall_B_computePointCloseToX", attackersDefaultPositions.at(i).x);
        bb.SetDouble("ReceiveBall_B_computePointCloseToY", attackersDefaultPositions.at(i).y);
        bb.SetBool("ReceiveBall_B_setSignal", true);
        bb.SetBool("ReceiveBall_B_shouldFail", true);
        bb.SetDouble("ReceiveBall_B_acceptableDeviation", 0.5);

        // bb.SetBool("ReceiveBall_A_shootAtGoal", false);

        // Create message
        rd.tree = "rtt_jim/IndependentAttackerRole"; //"rtt_jelle/GeneralAttacker";
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

	void Jim_IndependentAttackersPlay::reInitializeWhenNeeded(){
		// First, check if there are any robots available
		std::vector<int> robotsAvailable = getAvailableRobots();

		if((int)robotsAvailable.size() == 0)
			return;

		// There are robots available! Release all the robots we previously claimed, and then claim all available robots again.
		release_robots(robotsClaimed);
        robotsClaimed.clear();
        robotsAvailable = getAvailableRobots();

        /// For each available robot, initialize it as an attacker
        int numAttackers = (int)robotsAvailable.size();
        ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Initializing " << numAttackers << " strikers");

		roboteam_msgs::World world = LastWorld::get();
        roboteam_msgs::GeometryFieldSize geom = LastWorld::get_field();
        float circleRadius = geom.field_width * 0.25;
        Vector2 circlePos = Vector2(LastWorld::get_their_goal_center());

        // Divide the strikers over the field
        std::vector<Vector2> attackPositions = RobotPatternGenerator::Circle(numAttackers, M_PI/2, circleRadius, circlePos, M_PI, 0, -1.0);
		// Find out which robots is the closest to which position
		std::vector<int> robotsToPositions = Jim_MultipleDefendersPlay::assignRobotsToPositions(robotsAvailable, attackPositions, world);

		// Get the default roledirective publisher
		auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

		for (int i = 0; i < numAttackers; i++) {

			int strikerID = robotsToPositions.at(i);
			delete_from_vector(robotsAvailable, strikerID);
			claim_robot(strikerID);

			bt::Blackboard bb;

			// Set the robot ID
			bb.SetInt("ROBOT_ID", strikerID);
			bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());

			// bb.SetBool("ReceiveBall_B_computePoint", true);
			bb.SetDouble("ReceiveBall_A_acceptableDeviation", 1.5);
			bb.SetDouble("ReceiveBall_B_computePointCloseToX", attackPositions.at(i).x);
			bb.SetDouble("ReceiveBall_B_computePointCloseToY", attackPositions.at(i).y);
			bb.SetBool("ReceiveBall_B_setSignal", true);
			bb.SetBool("ReceiveBall_B_shouldFail", true);
			bb.SetDouble("ReceiveBall_B_acceptableDeviation", 0.5);
			// bb.SetBool("ReceiveBall_A_shootAtGoal", false);

			// Create message
			roboteam_msgs::RoleDirective rd;
			rd.robot_id = strikerID;
			rd.tree = "rtt_jim/IndependentAttackerRole"; //"rtt_jelle/GeneralAttacker";
			rd.blackboard = bb.toMsg();

			// Add random token and save it for later
			boost::uuids::uuid token = unique_id::fromRandom();
			tokens.push_back(token);
			rd.token = unique_id::toMsg(token);

			// Send to rolenode
			pub.publish(rd);

			ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "    robot " << strikerID << " is now a striker");
		}

	}

bt::Node::Status Jim_IndependentAttackersPlay::Update() {

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

	reInitializeWhenNeeded();

    return Status::Running;
}

} // rtt
