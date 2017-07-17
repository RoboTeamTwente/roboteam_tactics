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
#include "roboteam_msgs/WorldBall.h"

#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/conditions/WeHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_tactics/skills/SimpleDefender.h"

#include "roboteam_utils/LastWorld.h"


#define RTT_CURRENT_DEBUG_TAG Jim_MultipleDefendersPlay

namespace rtt {

RTT_REGISTER_TACTIC(Jim_MultipleDefendersPlay);

Jim_MultipleDefendersPlay::Jim_MultipleDefendersPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

boost::optional<int> Jim_MultipleDefendersPlay::getClosestDefender(std::vector<int> robots,
		roboteam_msgs::World& world, Vector2 dangerPos, double angleOffset) {
    double distanceFromGoal = 1.35;
    Vector2 defensePoint = SimpleDefender::computeDefensePoint(dangerPos, true, distanceFromGoal, angleOffset);
    auto defenderID = get_robot_closest_to_point(robots, world, defensePoint);
    return defenderID;
}

std::vector<int> Jim_MultipleDefendersPlay::getClosestRobots(std::vector<int> robots, std::vector<Vector2> points, roboteam_msgs::World& world) {
    
    // If the number of points is larger than the number of robots, choose the first points to drive to
	if (points.size() > robots.size()) {
		points.resize(robots.size());
	}

    if (points.size() == 0) {
        std::vector<int> emptyVec; 
        return emptyVec;
    }

    // Make a vector containing all the robots in the WorldRobot message type
    std::vector<roboteam_msgs::WorldRobot> worldRobots;
    for (size_t i = 0; i < robots.size(); i++) {
        boost::optional<roboteam_msgs::WorldRobot> robot = getWorldBot(robots.at(i), true, world);
        if (robot) {
            worldRobots.push_back(*robot);
        }
    }

    // Make a matrix (vector (y) of a vector (x)) containing the distances between the robots and the points
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


// void Jim_MultipleDefendersPlay::reInitialize(int newNumBallDefenders, int newNumRobotDefenders, std::vector<roboteam_msgs::WorldRobot> dangerousOpps) {
bool Jim_MultipleDefendersPlay::reInitializeWhenNeeded(bool justChecking) {
	// time_point startInit = now();

	roboteam_msgs::World world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);
    Vector2 ballVel(world.ball.vel);
    Vector2 ourGoalPos = LastWorld::get_our_goal_center();

    std::vector<int> robots = RobotDealer::get_available_robots();
    int numRobots = robots.size() + activeRobots.size();
    ROS_INFO("available robots: %i, activeRobots %i, numRobots: %i",robots.size(),activeRobots.size(),numRobots);

       
    if (numRobots < 1) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return false;
    }
    
    int minBallDefenders = 1;
    int maxBallDefenders = 2;
    double minDangerScore;
    std::vector<double> distancesBallDefendersFromGoal;
    distancesBallDefendersFromGoal.clear();

    bool ballOnTheirSide = ballPos.x > 0.0 || ballVel.x > 0.5;

    auto bb = std::make_shared<bt::Blackboard>();
    WeHaveBall weHaveBall("", bb);
    bool weAreAttacking = ballOnTheirSide || (weHaveBall.Update() == Status::Success);

    
    // Filter removed because it was not working: when it changed to atacking / defending, the number of ball or robot defenders changed, tricking the full reinitialize. This resets the filter and loop continuous

    if (weAreAttacking) {
    	minDangerScore = 8.0;
    	distancesBallDefendersFromGoal.push_back(1.35);
    	distancesBallDefendersFromGoal.push_back(3.00);
    } else {
    	minDangerScore = 3.2;
    	distancesBallDefendersFromGoal.push_back(1.35);
    	distancesBallDefendersFromGoal.push_back(1.35);
    }

    ROS_INFO("minDangerScore: %f",minDangerScore);

    std::vector<roboteam_msgs::WorldRobot> dangerousOpps;
    for (size_t i = 0; i < world.dangerList.size(); i++) {
        ROS_INFO("opponent %i, danger score: %f",i,world.dangerScores.at(i));
        if (world.dangerScores.at(i) >= minDangerScore) {
            roboteam_msgs::WorldRobot opp = world.dangerList.at(i);
            double angleDiffBall = fabs(cleanAngle((Vector2(opp.pos) - ourGoalPos).angle() - (ballPos - ourGoalPos).angle()));
            if (angleDiffBall <= 0.15) {
                minBallDefenders = 2;
            } else {

            	bool addDangerousOpp = true;
            	for (size_t j = 0; j < dangerousOpps.size(); j++) {
            		double angleDiffRobot = fabs(cleanAngle((Vector2(opp.pos) - ourGoalPos).angle() - (Vector2(dangerousOpps.at(j).pos) - ourGoalPos).angle()));
            		if (angleDiffRobot <= 0.15) {
            			addDangerousOpp = false;
            			break;
            		}
            	}
            	
            	if (addDangerousOpp) {
                    dangerousOpps.push_back(opp);
            	}
            }
        }
        
    }
    int numDangerousOpps = dangerousOpps.size();
    ROS_INFO("!!!numDangerousOpps:%i",numDangerousOpps);
 
    int newNumBallDefenders = std::min(numRobots, minBallDefenders); // start with a number of ball defenders
   int newNumRobotDefenders = std::min(numDangerousOpps, numRobots - newNumBallDefenders); // limit robot defenders to dangerous opps or to available robots
    newNumBallDefenders = std::max(newNumBallDefenders, numRobots - newNumRobotDefenders); // maximize the amount of ball defenders to the amount of available robots
    newNumBallDefenders = std::min(newNumBallDefenders, maxBallDefenders); // max 2 ball defenders


    if (justChecking) {
	    if (newNumBallDefenders != numBallDefenders || newNumRobotDefenders != numRobotDefenders ) {
	        ROS_INFO("new number of defenders not equal to old numbers of defenders: newBallDef:%i, BallDef:%i;  newRobotDef:%i, robotDef:%i",newNumBallDefenders,numBallDefenders,newNumRobotDefenders,numRobotDefenders);
            return true;
        } else {
            return false;
	    }
    }
    

    numBallDefenders = newNumBallDefenders;
	numRobotDefenders = newNumRobotDefenders;

	RTT_DEBUGLN("Initializing numBallDef: %i, numRobotDef: %i", numBallDefenders, numRobotDefenders); 
	
	activeRobots.clear();

	int keeperID = RobotDealer::get_keeper();

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // =========================
    // Initialize the Keeper
    // =========================
    {
        // RTT_DEBUGLN("Initializing Keeper %i", keeperID);
        delete_from_vector(robots, keeperID);
        ROS_INFO("claim on line 231");
        claim_robot(keeperID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = keeperID;
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
    
    if (weAreAttacking) {
    	if (numBallDefenders == 1) {
	        angleOffsets.push_back(0.0);
	    } else if (numBallDefenders == 2) {
	    	if (ballPos.y >= 0) {
	    		angleOffsets.push_back(0.15);
	        	angleOffsets.push_back(-0.05);
	    	} else {
	    		angleOffsets.push_back(-0.15);
	        	angleOffsets.push_back(0.05);
	    	}
	    }
    } else {
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
    }
    

    std::vector<Vector2> ballDefendersPositions;
    for (int i = 0; i < numBallDefenders; i++) {
        ballDefendersPositions.push_back(SimpleDefender::computeDefensePoint(world.ball.pos, true, distancesBallDefendersFromGoal.at(i), angleOffsets.at(i)));
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
        ROS_INFO_STREAM("robot: " << ballDefenderID << " distance: " << distancesBallDefendersFromGoal.at(i));
        bb.SetDouble("SimpleDefender_A_distanceFromGoal", distancesBallDefendersFromGoal.at(i));
        bb.SetDouble("SimpleDefender_A_angleOffset", angleOffsets.at(i));
        bb.SetBool("SimpleDefender_A_avoidRobots", false);
        bb.SetBool("SimpleDefender_A_dontDriveToBall", true);
        bb.SetBool("SimpleDefender_A_avoidBallsFromOurRobots", true);

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

    numBallDefenders = newNumBallDefenders;


    // ==================================
    // Initialize the Robot Defenders!
    // ==================================
    for (int i = 0; i < numRobotDefenders; i++) {

        roboteam_msgs::WorldRobot mostDangerousRobot = dangerousOpps.at(i);
        if(robots.size()>0){

            int defenderID = *getClosestDefender(robots, world, Vector2(mostDangerousRobot.pos), 0.0);

            // RTT_DEBUGLN("Initializing Robot Defender %i", defenderID);
            delete_from_vector(robots, defenderID);
            claim_robot(defenderID);

            roboteam_msgs::RoleDirective rd;
            rd.robot_id = defenderID;
            activeRobots.push_back(defenderID);
            bt::Blackboard bb;

            // Set the robot ID
            bb.SetInt("ROBOT_ID", defenderID);
            bb.SetInt("KEEPER_ID", keeperID);

            bb.SetInt("SimpleDefender_A_defendRobot", mostDangerousRobot.id);
            bb.SetDouble("SimpleDefender_A_distanceFromGoal", 1.35);
            bb.SetBool("SimpleDefender_A_avoidBallsFromOurRobots", true);

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
        else {
            ROS_ERROR("there was a mistake in determining the number of defenders to use");
        }
    }

    // double timeLapsed = time_difference_milliseconds(startInit, now()).count();

    return false;
}


void Jim_MultipleDefendersPlay::Initialize() {
    activeRobots.clear();

	numBallDefenders = 0;
	numRobotDefenders = 0;
	distBallToGoalThreshold = 4.0;
	weAreAttackingCounter = 0;
    weWereAttacking = false;
    reInitializeWhenNeeded(false);
	return;
}


bt::Node::Status Jim_MultipleDefendersPlay::Update() {

	if (reInitializeWhenNeeded(true)) {
        RTT_DEBUGLN_TEAM("Should reInitialize!");
        return Status::Failure;
    } else {
        return Status::Running;
    }
    
}

} // rtt
