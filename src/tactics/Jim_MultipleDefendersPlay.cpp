#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_tactics/skills/SimpleKeeper.h"

#include "roboteam_utils/LastWorld.h"


#define RTT_CURRENT_DEBUG_TAG Jim_MultipleDefendersPlay

namespace rtt {

RTT_REGISTER_TACTIC(Jim_MultipleDefendersPlay);

Jim_MultipleDefendersPlay::Jim_MultipleDefendersPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

int Jim_MultipleDefendersPlay::getClosestDefender(std::vector<int> robots, roboteam_msgs::World& world, Vector2 dangerPos, double angleOffset) {
    double distanceFromGoal = 1.35;
    Vector2 defensePoint = SimpleKeeper::computeDefensePoint(dangerPos, true, distanceFromGoal, 0.0);
    int defenderID = get_robot_closest_to_point(robots, world, defensePoint);
    return defenderID;
}

void Jim_MultipleDefendersPlay::Initialize() {
    tokens.clear();
    roboteam_msgs::World world = LastWorld::get();

    RTT_DEBUGLN_TEAM("Initializing Jim_MultipleDefendersPlay");    
    if (RobotDealer::get_available_robots().size() < 2) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = RobotDealer::get_available_robots();
    activeRobots.clear();
    
    int keeperID = RobotDealer::get_keeper();

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the Keeper
    {
        RTT_DEBUGLN("Initializing Keeper %i", keeperID);
        delete_from_vector(robots, keeperID);
        RobotDealer::claim_robot(keeperID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = keeperID;
        activeRobots.push_back(keeperID);
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

    int numBallDefenders = std::min((int) robots.size(), 2);
    std::vector<double> angleOffsets;
    if (numBallDefenders == 2) {
        angleOffsets.push_back(0.1);
        angleOffsets.push_back(-0.1);
    }
    RTT_DEBUGLN("numBallDefenders: %i", numBallDefenders);

    for (int i = 0; i < numBallDefenders; i++) {
        int ballDefenderID = getClosestDefender(robots, world, Vector2(world.ball.pos), angleOffsets.at(i));

        RTT_DEBUGLN("Initializing BallDefender %i", ballDefenderID);
        delete_from_vector(robots, ballDefenderID);
        RobotDealer::claim_robot(ballDefenderID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = ballDefenderID;
        activeRobots.push_back(ballDefenderID);
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", ballDefenderID);
        bb.SetInt("KEEPER_ID", keeperID);

        bb.SetDouble("DistanceXToY_A_distance", 2.0);
        bb.SetDouble("SimpleKeeper_A_distanceFromGoal", 1.35);
        bb.SetDouble("SimpleKeeper_A_angleOffset", angleOffsets.at(i));
        bb.SetBool("SimpleKeeper_A_avoidRobots", false);

        // Create message
        rd.tree = "rtt_jim/AggDefRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }

    for (size_t i = 0; i < robots.size(); i++) {
        std::cout << robots.at(i) << " ";
    }
    std::cout << " \n";

    int numRobotDefenders = std::min((int) robots.size(), 3);
    int numDangerousOpps = 0;
    for (size_t i = 0; i < world.dangerList.size(); i++) {
        if (world.dangerScores.at(i) >= 3.2) {
            numDangerousOpps++;
        }
    }
    numRobotDefenders = std::min(numRobotDefenders, numDangerousOpps);
    RTT_DEBUGLN("numRobotDefenders: %i", numRobotDefenders);


    // Create the robot defender roles
    for (int i = 0; i < numRobotDefenders; i++) {

        roboteam_msgs::WorldRobot mostDangerousRobot = world.dangerList.at(i);
        int defenderID = getClosestDefender(robots, world, Vector2(mostDangerousRobot.pos), 0.0);

        RTT_DEBUGLN("Initializing Defender %i", defenderID);
        delete_from_vector(robots, defenderID);
        RobotDealer::claim_robot(defenderID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = defenderID;
        activeRobots.push_back(defenderID);
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", defenderID);
        bb.SetInt("KEEPER_ID", keeperID);

        bb.SetInt("SimpleKeeper_A_defendRobot", mostDangerousRobot.id);
        bb.SetDouble("SimpleKeeper_A_distanceFromGoal", 1.35);

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


    ROS_INFO_STREAM("numLeftOver: " << robots.size());
    int robotsLeftOver = robots.size();

    for (int i = 0; i < robotsLeftOver; i++) {
        int leftOverID = robots.at(i);

        // RTT_DEBUGLN("Initializing BallDefender %i", ballDefenderID);
        // ROS_INFO_STREAM("leftover " << leftOverID << " gotox " << -1.0 - (0.3*leftOverID));
        // delete_from_vector(robots, leftOverID);
        RobotDealer::claim_robot(leftOverID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = leftOverID;
        activeRobots.push_back(leftOverID);
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", leftOverID);
        bb.SetInt("KEEPER_ID", keeperID);

        bb.SetDouble("GoToPos_A_xGoal", -0.5 - (0.3*leftOverID));
        bb.SetDouble("GoToPos_A_yGoal", 3.2);
        bb.SetDouble("GoToPos_A_angleGoal", 0.0);
        bb.SetBool("GoToPos_A_avoidRobots", true);

        // Create message
        rd.tree = "rtt_jim/GoToPosRole";
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

void Jim_MultipleDefendersPlay::ReleaseAllBots() {

    ROS_INFO_STREAM("Jim_MultipleDefendersPlay Terminating");

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    for (size_t i = 0; i < activeRobots.size(); i++) {
        roboteam_msgs::RoleDirective rd;
        rd.robot_id = activeRobots.at(i);
        rd.tree = roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE;
        pub.publish(rd);
        RobotDealer::release_robot(activeRobots.at(i));
    }

    activeRobots.clear();
    return;
}

bt::Node::Status Jim_MultipleDefendersPlay::Update() {

    if (time_difference_milliseconds(start, now()).count() >= 5000) {
        ReleaseAllBots();
        RTT_DEBUGLN("Reinitializing Jim_MultipleDefendersPlay...");
        Initialize();
    }

    // if ()

    // roboteam_msgs::World world = LastWorld::get();
    // dangerOrder = world.dangerList;
    // if (dangerOrder.size() >= 1 && prevDangerOrder.size() >= 1) {
    //     if (dangerOrder.at(0).id != prevDangerOrder.at(0).id) {
    //         Stop();
    //         Initialize();
    //     }
    // }
   
    // prevDangerOrder = dangerOrder;

    // int successCount = 0;

    // for (auto token : tokens) {
    //     if (feedbacks.find(token) != feedbacks.end()) {
    //         Status status = feedbacks.at(token);
    //         if (status == bt::Node::Status::Success) {
    //             successCount++;
    //         }
    //     }
    // }

    // if (successCount >= 2) {
    //     RTT_DEBUGLN("Jim_MultipleDefendersPlay succeeded!");
    //     return Status::Success;
    // }

    // auto duration = time_difference_seconds(start, now());
    // if (duration.count() >= 25) {
    //     return Status::Failure;
    // }

    return Status::Running;
}

} // rtt