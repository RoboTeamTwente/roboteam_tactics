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

#include "roboteam_tactics/tactics/Anouk_MultipleDefendersPlay.h"
#include "roboteam_tactics/conditions/WeHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_tactics/skills/SimpleDefender.h"

#include "roboteam_utils/LastWorld.h"


#define RTT_CURRENT_DEBUG_TAG Anouk_MultipleDefendersPlay

namespace rtt {

    RTT_REGISTER_TACTIC(Anouk_MultipleDefendersPlay);

    Anouk_MultipleDefendersPlay::Anouk_MultipleDefendersPlay(std::string name, bt::Blackboard::Ptr blackboard)
            : Tactic(name, blackboard)
    {}

    boost::optional<int> Anouk_MultipleDefendersPlay::getClosestDefender(std::vector<int> robots,
                                                                       roboteam_msgs::World& world, Vector2 dangerPos, double angleOffset) {
        double distanceFromGoal = 1.35;
        Vector2 defensePoint = SimpleDefender::computeDefensePoint(dangerPos, true, distanceFromGoal, angleOffset);
        boost::optional<int> defenderID = get_robot_closest_to_point(robots, world, defensePoint);

        if (defenderID) {
            return *defenderID;
        } else {
            ROS_WARN("Found no defender");
            return boost::none;
        }

    }

    namespace {

// Calculates the length of the vector from each robot position to each point
// and sums it
        double calcTotalCostAnouk(std::map<int, Vector2> const & currentPositions, std::vector<int> const & robots, std::vector<Vector2> const & points) {
            double total = 0;
            for (size_t i = 0; i < points.size(); ++i) {
                total += currentPositions.at(robots[i]).dist(points[i]);
            }
            return total;
        }

    }

    std::vector<int> Anouk_MultipleDefendersPlay::assignRobotsToPositions(std::vector<int> robots,
                                                                        std::vector<Vector2> points, roboteam_msgs::World& world) {
        // If the number of points is larger than the number of robots, choose the first points to drive to
        if (points.size() > robots.size()) {
            points.resize(robots.size());
        }

        // If there are no points nor robots return the empty list
        if (points.size() == 0) {
            return {};
        }

        // Cache robot positions or kick them out of the list if they can't be found
        std::map<int, Vector2> currentPositions;
        for (auto it = robots.begin(); it != robots.end();) {
            if (auto botOpt = getWorldBot(*it, true, world)) {
                currentPositions[*it] = botOpt->pos;
                it++;
            } else {
                it = robots.erase(it);
                points.pop_back();
            }
        }

        // Sort the robots so it's a base case that the while loops stops at
        std::sort(robots.begin(), robots.end());

        std::vector<int> minAssignment = robots;
        double minCost = calcTotalCostAnouk(currentPositions, robots, points);

        while (std::next_permutation(robots.begin(), robots.end())) {
            double candidateCost = calcTotalCostAnouk(currentPositions, robots, points);
            if (candidateCost < minCost) {
                minAssignment = robots;
                minCost = candidateCost;
            }
        }

        if (minAssignment.size() > points.size()) {
            minAssignment.resize(points.size());
        }

        return minAssignment;
    }

    bool Anouk_MultipleDefendersPlay::reInitializeWhenNeeded(bool justChecking) {

//        ROS_INFO_STREAM_NAMED("plays.Anouk", "================");

        // Save some values
        roboteam_msgs::World world = LastWorld::get();
        Vector2 ballPos(world.ball.pos);
        Vector2 ballVel(world.ball.vel);
        Vector2 ourGoalPos = LastWorld::get_our_goal_center();

        // Check if any previously assigned robots have vanished
        activeRobots.erase(std::remove_if(activeRobots.begin(), activeRobots.end(), [&world](int id) {
            return !getWorldBot(id, true, world);
        }), activeRobots.end());

        // Save how many robots available/active and how many robots in total
        std::vector<int> robots = getAvailableRobots();
        int numRobots = robots.size() + activeRobots.size();
        int totalNumRobots = world.us.size();

        // We can use max 5 of all robots
        if (numRobots >= (totalNumRobots - 1)) {
            numRobots = totalNumRobots - 2;
        }

        // If less then one robot is available throw an error
        if (numRobots < 1) {
            ROS_INFO_STREAM_THROTTLE_NAMED(1, "Anouk_MultipleDefendersPlay", "Not enough robots, cannot initialize... \n");
            // TODO: Want to pass failure here as well!
            return false;
        }

        // Set min and max number of ball defenders
        int minBallDefenders = 2;
        int maxBallDefenders = 2;
        int maxExtraDefenders = 2;


        double minDangerScore;
        std::vector<double> distancesBallDefendersFromGoal;

        bool ballOnTheirSide = ballPos.x > 0.0; //|| ballVel.x > 0.5;

        auto bb = std::make_shared<bt::Blackboard>();
        WeHaveBall weHaveBall("", bb);

        bool weAreAttacking = ballOnTheirSide; // && (weHaveBall.Update() == Status::Success);



        /* Draw box around field */
        if(weAreAttacking) drawer.setColor(0, 0, 255);
        else               drawer.setColor(255, 0, 0);

        // top left to top right
        Vector2 topleft(-6, 4.5);
        Vector2 bottomright(6, -4.5);

        drawer.drawLine("topleft_to_topright",       topleft, Vector2(12,0));
        drawer.drawLine("topleft_to_bottomleft",     topleft, Vector2(0, -9));

        drawer.drawLine("bottomright_to_bottomleft", bottomright, Vector2(-12,0));
        drawer.drawLine("bottomright_to_topright",   bottomright, Vector2(0, 9));

        drawer.setColor(0, 0, 0);
        /*    End of draw box    */



        if(weAreAttacking != weWereAttacking){
            ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", "Switched to " << (weAreAttacking ? "attacking" : "defending"));
        }

        // Set the distance BallDefenders from goal values and minDangerscore for weareattacking and not attacking
        if (weAreAttacking) {
            maxExtraDefenders = 0;
            minDangerScore = 8.0; // 8.0
            distancesBallDefendersFromGoal.push_back(1.35);
            distancesBallDefendersFromGoal.push_back(1.35); // 3.00
        } else {
            maxExtraDefenders = 2;
            minDangerScore = 1.0; //3.2
            distancesBallDefendersFromGoal.push_back(1.35);
            distancesBallDefendersFromGoal.push_back(1.35);
        }


        // Add dangerous opponents to a dangerlist
        std::vector<roboteam_msgs::WorldRobot> dangerousOpps;
        for (size_t i = 0; i < world.dangerList.size(); i++) {
            if (world.dangerScores.at(i) >= minDangerScore) {
                roboteam_msgs::WorldRobot opp = world.dangerList.at(i);
                double angleDiffBall = fabs(cleanAngle((Vector2(opp.pos) - ourGoalPos).angle() - (ballPos - ourGoalPos).angle()));
                if (angleDiffBall <= 0.15) {
                    ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", "setting minBallDefenders=2, for robot=" << opp.id);
                    minBallDefenders = 2;
                } else {
                    bool addDangerousOpp = true;
                    for (size_t j = 0; j < dangerousOpps.size(); j++) {
                        double angleDiffRobot = fabs(cleanAngle((Vector2(opp.pos) - ourGoalPos).angle() - (Vector2(dangerousOpps.at(j).pos) - ourGoalPos).angle()));
                        if (angleDiffRobot <= 0.15) {
                            ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", "Not adding robotDefender for robot=" << opp.id);
                            addDangerousOpp = false;
                            break;
                        }
                    }

                    if (addDangerousOpp) {
                        ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", "Adding robotDefender for robot=" << opp.id);
                        dangerousOpps.push_back(opp);
                    }
                }
            }

        }
        int numDangerousOpps = dangerousOpps.size();
//        std::cout << "numDangerousOpps = " << numDangerousOpps << "!!!!!" << std::endl;


        int newNumBallDefenders = std::min(numRobots, minBallDefenders); // start with a number of ball defenders
        int newNumRobotDefenders = std::min(numDangerousOpps, numRobots - newNumBallDefenders); // limit robot defenders to dangerous opps or to available robots
        int newNumExtraDefenders = std::max(numExtraDefenders, numRobots - newNumRobotDefenders - newNumBallDefenders); // maximize the amount of ball defenders to the amount of available robots
        newNumExtraDefenders = std::min(newNumExtraDefenders, maxExtraDefenders);


        ROS_INFO_STREAM_THROTTLE_NAMED(1, "Anouk_MultipleDefendersPlay", "numExtraDef: " << numExtraDefenders << ", newNumExtraDef: " << newNumExtraDefenders);
//        std::cout << "numExtraDef = " << numExtraDefenders << "!!!!! newNumExtraDef = " << newNumExtraDefenders << "!!!!!" << std::endl;




//        newNumBallDefenders = std::max(newNumBallDefenders, numRobots - newNumRobotDefenders); // maximize the amount of ball defenders to the amount of available robots
//        newNumBallDefenders = std::min(newNumBallDefenders, maxBallDefenders); // max 2 ball defenders

        // when it needs to check(update) it looks if the number of balldefenders or robotdefenders changed, when it did we should reinitialize
        if (justChecking) {
            if (weAreAttacking != weWereAttacking) {
                weWereAttacking = weAreAttacking;
//                 return true;
            }
            weWereAttacking = weAreAttacking;
            return newNumBallDefenders != numBallDefenders || newNumRobotDefenders != numRobotDefenders || newNumExtraDefenders != numExtraDefenders;
//            return newNumExtraDefenders != numExtraDefenders;
        }



        ROS_INFO_STREAM_NAMED("plays.Anouk", "=====================");


        weWereAttacking = weAreAttacking;

        numBallDefenders = newNumBallDefenders;
        numRobotDefenders = newNumRobotDefenders;
        numExtraDefenders = newNumExtraDefenders;

        ROS_INFO_STREAM_THROTTLE_NAMED(1, "Anouk_MultipleDefendersPlay", "numBallDef: " << numBallDefenders << ", numRobotDef: " << numRobotDefenders);

        activeRobots.clear();

        int keeperID = RobotDealer::get_keeper();

        // Get the default roledirective publisher
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

        // =========================
        // Initialize the Keeper
        // =========================
        {
            delete_from_vector(robots, keeperID);

            claim_robot(keeperID);

            roboteam_msgs::RoleDirective rd;
            rd.robot_id = keeperID;
            bt::Blackboard bb;

            bb.SetInt("ROBOT_ID", keeperID);
            bb.SetInt("KEEPER_ID", keeperID);

            // Create message
            rd.tree = "rtt_jim/KeeperRole";
            rd.blackboard = bb.toMsg();

            // Add random token and save it for later
            boost::uuids::uuid token = unique_id::fromRandom();
            tokens.push_back(token);
            rd.token = unique_id::toMsg(token);

            // Send to rolenode
            ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", rd.robot_id << " -> " << rd.tree);
            ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", "bb: " << bb.toString());
            pub.publish(rd);
        }


        // ====================================
        // Initialize the Ball Defenders!
        // ====================================
        std::vector<double> angleOffsets;
        std::vector<Vector2> idlePositions;

        // positions for the ball defenders when ball is in defense area
        idlePositions.push_back(Vector2(-4.8, 1.15));
        idlePositions.push_back(Vector2(-4.8, -1.15));

        if (weAreAttacking) {
            if (numBallDefenders == 1) {
                angleOffsets.push_back(0.0);
            } else if (numBallDefenders == 2) {
                if (ballPos.y >= 0) {
                    angleOffsets.push_back(0.15);
                    angleOffsets.push_back(-0.15);
                } else {
                    angleOffsets.push_back(-0.15);
                    angleOffsets.push_back(0.15);
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
        std::vector<int> ballDefenders = assignRobotsToPositions(robots, ballDefendersPositions, world);

        for (size_t i = 0; i < ballDefenders.size(); i++) {
            int ballDefenderID = ballDefenders.at(i);

            // RTT_DEBUGLN("Initializing BallDefender %i", ballDefenderID);
            delete_from_vector(robots, ballDefenderID);
            claim_robot(ballDefenderID);
            ROS_INFO_STREAM_NAMED("plays.Anouk", "Claimed ball defender. ID : " << ballDefenderID);

            roboteam_msgs::RoleDirective rd;
            rd.robot_id = ballDefenderID;
            activeRobots.push_back(ballDefenderID);

            bt::Blackboard bb;

//            std::cout << "Trying to access " << i << std::endl;
//            std::cout << "angleOffset of robot [" << ballDefenderID << "] = " << angleOffsets.at(i) << std::endl;

            // Set the robot ID
            bb.SetInt("ROBOT_ID", ballDefenderID);
            bb.SetInt("KEEPER_ID", keeperID);

            bb.SetDouble("GoToPos_A_xGoal", idlePositions.at(i).x);
            bb.SetDouble("GoToPos_A_yGoal", idlePositions.at(i).y);
            bb.SetDouble("GoToPos_A_angleGoal", 0.0);


            //----------improvement defenders----------
            bb.SetBool("GetBall_B_aimAwayFromTarget", true);
            bb.SetString("GetBall_B_aimAt", "ourgoal");
            bb.SetBool("GetBall_D_aimAwayFromTarget", true);
            bb.SetString("GetBall_D_aimAt", "ourgoal");
            //----------improvement defenders----------


            bb.SetDouble("DistanceXToY_A_distance", 2.0);
            //ROS_INFO_STREAM("robot: " << ballDefenderID << " distance: " << distancesBallDefendersFromGoal.at(i));
            bb.SetDouble("SimpleGoalDefender_A_distanceFromGoal", distancesBallDefendersFromGoal.at(i));
            bb.SetDouble("SimpleGoalDefender_A_angleOffset", angleOffsets.at(i));
            bb.SetBool("SimpleGoalDefender_A_avoidRobots", false);
            bb.SetBool("SimpleGoalDefender_A_dontDriveToBall", true);
            bb.SetBool("SimpleGoalDefender_A_avoidBallsFromOurRobots", true);
//            bb.SetBool("SimpleDefender_A_ballDefender", true);


            // Create message
            rd.tree = "rtt_jim/DefenderRoleBallDefenders";
            rd.blackboard = bb.toMsg();

            // Add random token and save it for later
            boost::uuids::uuid token = unique_id::fromRandom();
            tokens.push_back(token);
            rd.token = unique_id::toMsg(token);

            // Send to rolenode
            ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", rd.robot_id << " -> " << rd.tree);
            ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", "bb: " << bb.toString());
            pub.publish(rd);
//            std::cout << "All access stuff succesful" << std::endl;
        }

        numBallDefenders = newNumBallDefenders;


        // ==================================
        // Initialize the Robot Defenders!
        // ==================================
        ROS_INFO_STREAM_NAMED("plays.Anouk", "Initializing Robot Defenders");
        for (int i = 0; i < numRobotDefenders; i++) {

            roboteam_msgs::WorldRobot mostDangerousRobot = dangerousOpps.at(i);
            Vector2 mostDangerousRobotPos = Vector2(mostDangerousRobot.pos);
            double defenseDistanceFromGoal = ((mostDangerousRobotPos.x + 6) * 0.9);

            if(robots.size()>0){
                int defenderID = *getClosestDefender(robots, world, Vector2(mostDangerousRobot.pos), 0.0);

                // RTT_DEBUGLN("Initializing Robot Defender %i", defenderID);
                delete_from_vector(robots, defenderID);
                claim_robot(defenderID);
                ROS_INFO_STREAM_NAMED("plays.Anouk", "Claimed robot defender. ID : " << defenderID);

                roboteam_msgs::RoleDirective rd;
                rd.robot_id = defenderID;
                activeRobots.push_back(defenderID);
                bt::Blackboard bb;

                // Set the robot ID
                bb.SetInt("ROBOT_ID", defenderID);
                bb.SetInt("KEEPER_ID", keeperID);


                //----------improvement defenders----------
                bb.SetBool("GetBall_B_aimAwayFromTarget", true);
                bb.SetString("GetBall_B_aimAt", "ourgoal");
                bb.SetBool("GetBall_D_aimAwayFromTarget", true);
                bb.SetString("GetBall_D_aimAt", "ourgoal");
                //----------improvement defenders----------


                bb.SetInt("SimpleDefender_A_defendRobot", mostDangerousRobot.id);
                bb.SetDouble("SimpleDefender_A_distanceFromGoal", defenseDistanceFromGoal); //1.35
                bb.SetBool("SimpleDefender_A_avoidBallsFromOurRobots", true);

                // Create message
                rd.tree = "rtt_jim/DefenderRoleGetBall";
                rd.blackboard = bb.toMsg();

                // Add random token and save it for later
                boost::uuids::uuid token = unique_id::fromRandom();
                tokens.push_back(token);
                rd.token = unique_id::toMsg(token);

                // Send to rolenode
                ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", rd.robot_id << " -> " << rd.tree);
                ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", "bb: " << bb.toString());
                pub.publish(rd);
            }
            else {
                ROS_ERROR("there was a mistake in determining the number of defenders to use");
            }
        }

        // ==================================
        // Initialize the extra Defenders!
        // ==================================
        ROS_INFO_STREAM_NAMED("plays.Anouk", "Initializing extra Robot Defenders");
        for (int i = 0; i < numExtraDefenders; i++) {


            /*  als er robots over zijn voor verdediging, moeten deze extra defenders op een goede plek gaan staan
             *  zodat als ze toch nodig zijn dat ze dan in de buurt zijn.
             *  misschien wil je ook dat deze robots vrij staan?? voor nu vaste locatie.
             *
             *
             *
             *
             *
             *
             *
             */

            std::vector<Vector2> standardDefendPositions;

            standardDefendPositions.push_back(Vector2(-3.0, 2.5));
            standardDefendPositions.push_back(Vector2(-3.0, -2.5));


            if(robots.size()>0){
                ROS_INFO_STREAM_NAMED("plays.Anouk", "standardDefendPositions.at " << i << ", size : " << standardDefendPositions.size());
                int extraDefenderID = *get_robot_closest_to_point(robots, world, standardDefendPositions.at(i));

                ROS_INFO_STREAM_NAMED("plays.Anouk", "standardDefendPositions.at succeeded");
//                int extraDefenderID = *getClosestDefender(robots, world, standardDefendPositions.at(i), 0.0);

                // RTT_DEBUGLN("Initializing Robot Defender %i", defenderID);
                delete_from_vector(robots, extraDefenderID);
                claim_robot(extraDefenderID);
                ROS_INFO_STREAM_NAMED("plays.Anouk", "Claimed extra defender. ID : " << extraDefenderID);

                roboteam_msgs::RoleDirective rd;
                rd.robot_id = extraDefenderID;
                activeRobots.push_back(extraDefenderID);
                bt::Blackboard bb;

                // Set the robot ID
                bb.SetInt("ROBOT_ID", extraDefenderID);
                bb.SetInt("KEEPER_ID", keeperID);


                //----------improvement defenders----------
                bb.SetBool("GetBall_B_aimAwayFromTarget", true);
                bb.SetString("GetBall_B_aimAt", "ourgoal");
                bb.SetBool("GetBall_D_aimAwayFromTarget", true);
                bb.SetString("GetBall_D_aimAt", "ourgoal");
                //----------improvement defenders----------

                bb.SetDouble("DistanceXToY_A_distance", 2.0);
                ROS_INFO_STREAM_NAMED("plays.Anouk", "standardDefendPositions.at(i).x/y");
                bb.SetDouble("ReceiveBall_A_receiveBallAtX", standardDefendPositions.at(i).x);
                bb.SetDouble("ReceiveBall_A_receiveBallAtY", standardDefendPositions.at(i).y);


                // Create message
                rd.tree = "rtt_jim/DefenderRoleExtraDefender";
                rd.blackboard = bb.toMsg();

                // Add random token and save it for later
                boost::uuids::uuid token = unique_id::fromRandom();
                tokens.push_back(token);
                rd.token = unique_id::toMsg(token);

                // Send to rolenode
                ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", rd.robot_id << " -> " << rd.tree);
                ROS_INFO_STREAM_NAMED("Anouk_MultipleDefendersPlay", "bb: " << bb.toString());
                pub.publish(rd);
            }
            else {
                ROS_ERROR("there was a mistake in determining the number of defenders to use");
            }
        }




        ROS_INFO_STREAM_NAMED("plays.Anouk", "Active robots claimed: " << activeRobots.size());
        ROS_INFO_STREAM_NAMED("plays.Anouk", "Total robots claimed : " << get_claimed_robots().size());



        ROS_INFO_STREAM_NAMED("plays.Anouk", "Initializing of Robot Defenders finished");




        // double timeLapsed = time_difference_milliseconds(startInit, now()).count();

        return false;
    }


    void Anouk_MultipleDefendersPlay::Initialize() {
        activeRobots.clear();

        numBallDefenders = 0;
        numRobotDefenders = 0;
        numExtraDefenders = 0;
        distBallToGoalThreshold = 4.0;
        weAreAttackingCounter = 0;
        weWereAttacking = false;
        reInitializeWhenNeeded(false);
        return;
    }


    bt::Node::Status Anouk_MultipleDefendersPlay::Update() {

        if (reInitializeWhenNeeded(true)) {
            RTT_DEBUGLN_TEAM("Should reInitialize!");
            return Status::Failure;
        } else {
            return Status::Running;
        }

    }

} // rtt
