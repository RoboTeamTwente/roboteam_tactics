#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_KickOffDefense.h"
#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"



#define RTT_CURRENT_DEBUG_TAG Jim_KickOffDefense

namespace rtt {

RTT_REGISTER_TACTIC(Jim_KickOffDefense);

Jim_KickOffDefense::Jim_KickOffDefense(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard)
        , weHaveBall("", blackboard)
        {}



void Jim_KickOffDefense::Initialize() {
    tokens.clear();

    RTT_DEBUGLN_TEAM("Initializing Jim_KickOffDefense");    
    if (getAvailableRobots().size() < 1) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ourGoalPos = LastWorld::get_our_goal_center();
    Vector2 ballPos = world.ball.pos;
    std::vector<roboteam_msgs::WorldRobot> theirRobots = world.them;   

        
    std::vector<int> robots = getAvailableRobots();
    int keeperID = RobotDealer::get_keeper();


    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();




    // =========================
    // Initialize the Keeper
    // =========================
    {
        // RTT_DEBUGLN("Initializing Keeper %i", keeperID);
        // delete_from_vector(robots, keeperID);
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





    int numBallDefenders;
    std::vector<double> angleOffsets;
    std::vector<double> distanceOffsets;
    if ((int) getAvailableRobots().size() <= 4) {
        numBallDefenders = 3;

        angleOffsets.push_back(0.1);
        angleOffsets.push_back(-0.1);
        angleOffsets.push_back(0.0);

        distanceOffsets.push_back(1.5);
        distanceOffsets.push_back(1.5);
        distanceOffsets.push_back(2.5);

    } else {
        numBallDefenders = 4;

        angleOffsets.push_back(0.2);
        angleOffsets.push_back(0.0);
        angleOffsets.push_back(-0.2);
        angleOffsets.push_back(0.0);

        distanceOffsets.push_back(1.5);
        distanceOffsets.push_back(1.5);
        distanceOffsets.push_back(1.5);
        distanceOffsets.push_back(3.7);
    }




    // ====================================
    // Initialize the Ball Defenders!
    // ====================================

    numBallDefenders = std::min((int) robots.size(), numBallDefenders);

    std::vector<Vector2> ballDefendersPositions;
    for (int i = 0; i < numBallDefenders; i++) {
        ballDefendersPositions.push_back(SimpleDefender::computeDefensePoint(world.ball.pos, true, distanceOffsets.at(i), angleOffsets.at(i)));
    }
    

    std::vector<int> ballDefenders = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, ballDefendersPositions, world);

    for (size_t i = 0; i < ballDefenders.size(); i++) {

        int ballDefenderID = ballDefenders.at(i);

        // RTT_DEBUGLN("Initializing BallDefender %i", ballDefenderID);
        delete_from_vector(robots, ballDefenderID);
        claim_robot(ballDefenderID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = ballDefenderID;
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", ballDefenderID);
        bb.SetInt("KEEPER_ID", keeperID);

        bb.SetDouble("DistanceXToY_A_distance", 2.0);
        bb.SetDouble("SimpleDefender_A_distanceFromGoal", distanceOffsets.at(i));
        bb.SetDouble("SimpleDefender_A_angleOffset", angleOffsets.at(i));
        bb.SetBool("SimpleDefender_A_avoidRobots", false);
        bb.SetBool("SimpleDefender_A_dontDriveToBall", true);
        bb.SetBool("SimpleDefender_A_stayAwayFromBall", true);

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







    int numRobotDefenders = std::min((int) getAvailableRobots().size(), 2);
    
    // ==================================
    // Find the most dangerous opponents
    // ==================================
    std::vector<roboteam_msgs::WorldRobot> dangerousOpps;
    for (size_t i = 0; i < world.dangerList.size(); i++) {
        // if (world.dangerScores.at(i) >= minDangerScore) {
            roboteam_msgs::WorldRobot opp = world.dangerList.at(i);
            double angleDiffBall = fabs(cleanAngle((Vector2(opp.pos) - ourGoalPos).angle() - (ballPos - ourGoalPos).angle()));
            if (angleDiffBall <= 0.15) {
                // minBallDefenders = 2;
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
        // }
    }

    std::vector<Vector2> defenderPositions;

    double distanceFromGoal = 3.8;

    for (size_t i = 0; i < dangerousOpps.size(); i++) {
        Vector2 defensePoint = SimpleDefender::computeDefensePoint(Vector2(dangerousOpps.at(i).pos), true, distanceFromGoal, 0.0);    
        defenderPositions.push_back(defensePoint);
    }



    // ==================================
    // Initialize the Robot Defenders!
    // ==================================
    
    numRobotDefenders = std::min(numRobotDefenders, (int) dangerousOpps.size()); 

    std::vector<int> defenderIDs = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, defenderPositions, world);

    RTT_DEBUGLN("num KickOffDefenders: %i", numRobotDefenders);

    for (int i = 0; i < numRobotDefenders; i++) {

        roboteam_msgs::WorldRobot mostDangerousRobot = dangerousOpps.at(i);
        int defenderID = defenderIDs.at(i);
        RTT_DEBUGLN("KickOffDenfer with id: %i",defenderID);
        // RTT_DEBUGLN("Initializing Robot Defender %i", defenderID);
        delete_from_vector(robots, defenderID);
        claim_robot(defenderID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = defenderID;
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", defenderID);
        bb.SetInt("KEEPER_ID", keeperID);

        bb.SetInt("SimpleDefender_A_defendRobot", mostDangerousRobot.id);
        bb.SetDouble("SimpleDefender_A_distanceFromGoal", distanceFromGoal);
        bb.SetString("SimpleDefender_A_stayOnSide", "ourSide");
        bb.SetBool("SimpleDefender_A_stayAwayFromBall", true);
        bb.SetBool("SimpleDefender_A_dontDriveToBall", true);

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

    initTime = now();
    



    // ==================================================================
    // Initialize the last Ball Defender! Will also be the kick off taker
    // ==================================================================

    // if (getAvailableRobots().size() >= 1) {
    //     std::cout << "Initializing ball defender!\n";

    //     int ballDefenderID = getAvailableRobots().at(0);

    //     // RTT_DEBUGLN("Initializing BallDefender %i", ballDefenderID);
    //     delete_from_vector(robots, ballDefenderID);
    //     claim_robot(ballDefenderID);

    //     roboteam_msgs::RoleDirective rd;
    //     rd.robot_id = ballDefenderID;
    //     bt::Blackboard bb;

    //     // Set the robot ID
    //     bb.SetInt("ROBOT_ID", ballDefenderID);
    //     bb.SetInt("KEEPER_ID", keeperID);

    //     bb.SetDouble("DistanceXToY_A_distance", 2.0);
    //     bb.SetDouble("SimpleDefender_A_distanceFromGoal", 3.7);
    //     bb.SetBool("SimpleDefender_A_avoidRobots", false);
    //     bb.SetBool("SimpleDefender_A_dontDriveToBall", true);

    //     // Create message
    //     rd.tree = "rtt_jim/DefenderRole";
    //     rd.blackboard = bb.toMsg();

    //     // Add random token and save it for later
    //     boost::uuids::uuid token = unique_id::fromRandom();
    //     tokens.push_back(token);
    //     rd.token = unique_id::toMsg(token);

    //     // Send to rolenode
    //     pub.publish(rd);
    // } else {
    //     std::cout << "No robots available for the last Ball Defender!\n";
    // }
}


bt::Node::Status Jim_KickOffDefense::Update() {

    if (time_difference_milliseconds(initTime, now()).count() >= 1000) {
        RTT_DEBUGLN("ReInitializing Jim_KickOffDefense");
        return Status::Failure;
    }

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballVel(world.ball.vel);

    if (ballVel.length() > 0.5 && (HasBool("allowSuccess") && GetBool("allowSuccess"))) {
        ROS_INFO_STREAM("KickOff success because of moving ball!");
        return Status::Success;
    }

    return Status::Running;
}

} // rtt
