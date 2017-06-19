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
    Vector2 defensePoint = SimpleKeeper::computeDefensePoint(dangerPos, true, distanceFromGoal, angleOffset);
    int defenderID = get_robot_closest_to_point(robots, world, defensePoint);
    return defenderID;
}

std::vector<int> Jim_MultipleDefendersPlay::getClosestRobots(std::vector<int> robots, std::vector<Vector2> points, roboteam_msgs::World& world) {
    
    if (points.size() == 0) {
        std::vector<int> emptyVec; 
        return emptyVec;
    }

    std::vector<roboteam_msgs::WorldRobot> worldRobots;
    for (size_t i = 0; i < robots.size(); i++) {
        boost::optional<roboteam_msgs::WorldRobot> robot = getWorldBot(robots.at(i), true, world);
        if (robot) {
            worldRobots.push_back(*robot);
        }
    }

    std::vector< std::vector<double> > distances;
    // std::cout << "points: \n";
    for (size_t i = 0; i < points.size(); i++) {
        // std::cout << points.at(i).x << " " << points.at(i).y << " \n";
        std::vector<double> dists;
        for (size_t j = 0; j < worldRobots.size(); j++) {
            double dist = (Vector2(worldRobots.at(j).pos) - points.at(i)).length();
            dists.push_back(dist);
        }
        distances.push_back(dists);
    }

    std::vector<int> optimalCombination;
    double lowestDist = std::numeric_limits<double>::max();

    std::vector<int> testCombination;
    for (size_t i = 0; i < worldRobots.size(); i++) {
        testCombination.push_back(i);
    } 

    int prevNum = -1;
    do {
        if (testCombination.at(points.size()-1) != prevNum) {
            // std::cout << "trying combination: ";
            double totalDist = 0.0;
            for (size_t i = 0; i < points.size(); i++) {
                totalDist += distances.at(i).at(testCombination.at(i));
                // std::cout << " p" << i << " r" << worldRobots.at(testCombination.at(i)).id;
            }

            if (totalDist < lowestDist) {
                optimalCombination = testCombination;
                lowestDist = totalDist;
            }
            // std::cout << " dist: " << totalDist << " \n";
            prevNum = testCombination.at(points.size()-1);
        }
    } while (std::next_permutation(testCombination.begin(), testCombination.end()));

    std::vector<int> closestRobots;
    for (size_t i = 0; i < points.size(); i++) {
        // std::cout << "robot " << worldRobots.at(optimalCombination.at(i)).id << " to x: " << points.at(i).x << " y: " << points.at(i).y << " \n"; 
        closestRobots.push_back(worldRobots.at(optimalCombination.at(i)).id);
    }

    return closestRobots;
}


void Jim_MultipleDefendersPlay::Initialize() {
    time_point startInit = now();

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

    int numRobots = robots.size();

    int numDangerousOpps = 0;
    for (size_t i = 0; i < world.dangerList.size(); i++) {
        if (world.dangerScores.at(i) >= 3.2) {
            numDangerousOpps++;
        }
    }

    int numBallDefenders = std::min((int) robots.size(), 1); // start with max 1 ball defenders
    int numRobotDefenders = std::min(numDangerousOpps, (int) robots.size() - numBallDefenders); // limit robot defenders to dangerous opps or to available robots
    numBallDefenders = std::max(numBallDefenders, (int) robots.size() - numRobotDefenders); // maximize the amount of ball defenders to the amount of available robots
    numBallDefenders = std::min(numBallDefenders, 2); // max 3 ball defenders

    if ((numRobotDefenders + numBallDefenders) > robots.size()) {
    	ROS_WARN("number of robots bigger than available....");
    }

    RTT_DEBUGLN("numRobotDefenders: %i, numBallDefenders: %i", numRobotDefenders, numBallDefenders);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();



    // =========================
    // Initialize the Keeper
    // =========================
    {
        RTT_DEBUGLN("Initializing Keeper %i", keeperID);
        delete_from_vector(robots, keeperID);
        claim_robot(keeperID);

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


    // ====================================
    // Initialize the Ball Defenders!
    // ====================================
    std::vector<double> angleOffsets;
    
    if (numBallDefenders == 1) {
        angleOffsets.push_back(0.0);
    } else if (numBallDefenders == 2) {
        angleOffsets.push_back(0.1);
        angleOffsets.push_back(-0.1);
    } else if (numBallDefenders == 3) {
        angleOffsets.push_back(0.0);
        angleOffsets.push_back(0.2);
        angleOffsets.push_back(-0.2);
    }

    std::vector<Vector2> ballDefendersPositions;
    for (int i = 0; i < numBallDefenders; i++) {
        ballDefendersPositions.push_back(SimpleKeeper::computeDefensePoint(world.ball.pos, true, 1.35, angleOffsets.at(i)));
    }
    std::vector<int> ballDefenders = getClosestRobots(robots, ballDefendersPositions, world);

    for (size_t i = 0; i < ballDefenders.size(); i++) {
        int ballDefenderID = ballDefenders.at(i);

        // RTT_DEBUGLN("Initializing BallDefender %i", ballDefenderID);
        delete_from_vector(robots, ballDefenderID);
        claim_robot(ballDefenderID);

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
        rd.tree = "rtt_jim/DefenderRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }


    // ==================================
    // Initialize the Robot Defenders!
    // ==================================
    for (int i = 0; i < numRobotDefenders; i++) {

        roboteam_msgs::WorldRobot mostDangerousRobot = world.dangerList.at(i);
        int defenderID = getClosestDefender(robots, world, Vector2(mostDangerousRobot.pos), 0.0);

        // RTT_DEBUGLN("Initializing Defender %i", defenderID);
        delete_from_vector(robots, defenderID);
        claim_robot(defenderID);

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

    double timeLapsed = time_difference_milliseconds(startInit, now()).count();
    std::cout << "Initializing took: " << timeLapsed << " ms \n";

    start = now();
}

void Jim_MultipleDefendersPlay::ReleaseAllBots() {

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

    if (time_difference_milliseconds(start, now()).count() >= 1000) {
        // ReleaseAllBots();
        Terminate(Status::Running);
        Initialize();
    }

    return Status::Running;
}

} // rtt