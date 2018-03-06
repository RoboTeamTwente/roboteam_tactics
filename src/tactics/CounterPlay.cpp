#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>
#include <algorithm>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/CounterPlay.h"
#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"



#define RTT_CURRENT_DEBUG_TAG CounterPlay

namespace rtt {

RTT_REGISTER_TACTIC(CounterPlay);

CounterPlay::CounterPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard)
        , weHaveBall("", blackboard)
        {}



void CounterPlay::Initialize() {
    tokens.clear();

    claim_robot(RobotDealer::get_keeper());


    std::vector<int> robots = getAvailableRobots();

    // if (getAvailableRobots().size() < 1) {
    if (robots.size() < 3) {
        //RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);

    // RTT_DEBUGLN("Initializing CounterPlay robots: %i", numRobots);    

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();


    // Initialize other players

    std::vector<Vector2> robotDefaultPositions;
    
    // Counter
    robotDefaultPositions.push_back(Vector2(-1.5, 2.0));
    // Assister
    robotDefaultPositions.push_back(Vector2( 4.0, 2.5));
    // Scorer
    robotDefaultPositions.push_back(Vector2( 3.0, 0.0));

    std::vector<int> robotIDs = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, robotDefaultPositions, world); 

    for (size_t i = 0; i < (size_t) robotIDs.size(); i++) {

        //RTT_DEBUGLN("Initializing CounterPlayRobot %i", i);
        claim_robot(robotIDs.at(i));

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = robotIDs.at(i);
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", robotIDs.at(i));
        bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());

        bb.SetDouble("ReceiveBall_A_receiveBallAtX", robotDefaultPositions.at(i).x);
	    bb.SetDouble("ReceiveBall_A_receiveBallAtY", robotDefaultPositions.at(i).y);
	    bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", true);

	    // The 'last' robot shoots at the goal
	    if(i < robotIDs.size() - 1) {
	    	bb.SetString("GetBall_A_aimAt", "robot");
	    	bb.SetInt("GetBall_A_aimAtRobot", robotIDs.at(i+1));
	    	bb.SetBool("GetBall_A_ourTeam", true);
        } else {
        	bb.SetString("GetBall_A_aimAt", "theirgoal");
        	bb.SetBool("shootAtGoal",true);
        }
        bb.SetBool("GetBall_A_passOn", true);

        // Create message
        rd.tree = "rtt_anouk/ScorerRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }

    // RTT_DEBUGLN("Initializing CounterPlayRobotKeeper");
    

    roboteam_msgs::RoleDirective rd;
    rd.robot_id = RobotDealer::get_keeper();
    bt::Blackboard bb;

    // Set the robot ID
    bb.SetInt("ROBOT_ID", RobotDealer::get_keeper());
    bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());

    bb.SetString("GetBall_A_aimAt", "robot");
    bb.SetInt("GetBall_A_aimAtRobot", robotIDs.at(0));
    bb.SetBool("GetBall_A_ourTeam", true);

    ROS_INFO_STREAM("aim at robot: " << robotIDs.at(0));
    // Create message
    rd.tree = "rtt_jim/KeeperRole";
    rd.blackboard = bb.toMsg();

    // Add random token and save it for later
    boost::uuids::uuid token = unique_id::fromRandom();
    tokens.push_back(token);
    rd.token = unique_id::toMsg(token);

    // Send to rolenode
    pub.publish(rd);


    lastTimeWeHadBall = now();
}


bt::Node::Status CounterPlay::Update() {

    // bool allFailed = true;

    // if (tokens.size() == 0) {
    //     allFailed = false;
    // }

    // for (auto token : tokens) {
    //     if (feedbacks.find(token) != feedbacks.end()) {
    //         Status status = feedbacks.at(token);
    //         if (status == bt::Node::Status::Success) {
    //             RTT_DEBUGLN_TEAM("Strikers succeeded!");
    //             return Status::Success;
    //         }

    //         allFailed &= status == bt::Node::Status::Failure;
    //     } else {
    //         allFailed = false;
    //     }
    // }

    // if (allFailed) {
    //     RTT_DEBUGLN_TEAM("Strikers failed!");
    //     return Status::Failure;


    // }

    return Status::Running;
}

} // rtt
