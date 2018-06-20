#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>
#include <sstream>

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
#include "roboteam_tactics/utils/RobotsToDefendFinder.h"

#include "roboteam_utils/LastWorld.h"


#define RTT_CURRENT_DEBUG_TAG Anouk_MultipleDefendersPlay

#define ROS_LOG_NAME "plays.Anouk_MDP"

namespace rtt {

    RTT_REGISTER_TACTIC(Anouk_MultipleDefendersPlay);

    Anouk_MultipleDefendersPlay::Anouk_MultipleDefendersPlay(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard), weAreAttacking(false) {}

    boost::optional<int> Anouk_MultipleDefendersPlay::getClosestDefender(std::vector<int> robots, roboteam_msgs::World& world, Vector2 dangerPos, double angleOffset) {
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

    std::vector<int> Anouk_MultipleDefendersPlay::assignRobotsToPositions(std::vector<int> robots, std::vector<Vector2> points, roboteam_msgs::World& world) {

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




    bool Anouk_MultipleDefendersPlay::reInitializeWhenNeeded(bool forceInitialize) {

        // Save some values
        roboteam_msgs::World world = LastWorld::get();
        Vector2 ballPos(world.ball.pos);
        Vector2 ballVel(world.ball.vel);

        /// === Are we attacking or defending === ///
        // Check if an opponent has the ball
        bool opponentHasBall = false;
        for(roboteam_msgs::WorldRobot robot : world.them){
            // If we already know that a robot has the ball, there is no need to check the other robots
            if(opponentHasBall)
                continue;

            Vector2 robotPos(robot.pos);
            Vector2 robotVel(robot.vel);

            // Check if ball is in range of robot
            double distanceRobotToBall = (robotPos - ballPos).length();
            // If distance is larger than 10cm, the robot doesn't have the ball
            if(0.3 < distanceRobotToBall)
                continue;

            // Check if the ball has the same speed as the robot, and isn't just passing by
            double velDifference = (robotVel - ballVel).length();
            // If difference in velocity is larger than 1m/s, the robot does not have the ball
            if(1 < velDifference)
                continue;

            // The robot has the ball
            opponentHasBall = true;
//            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Opponent " << robot.id << " has the ball. distanceRobotToBall=" << distanceRobotToBall << ", velDifference=" << velDifference);
        }
        // Check if we have the ball
        bool weHaveBall = false;
        for(roboteam_msgs::WorldRobot robot : world.us){
            // If we already know that a robot has the ball, there is no need to check the other robots
            if(weHaveBall)
                continue;

            Vector2 robotPos(robot.pos);
            Vector2 robotVel(robot.vel);

            // Check if ball is in range of robot
            double distanceRobotToBall = (robotPos - ballPos).length();
            // If distance is larger than 10cm, the robot doesn't have the ball
            if(0.3 < distanceRobotToBall)
                continue;

            // Check if the ball has the same speed as the robot, and isn't just passing by
            double velDifference = (robotVel - ballVel).length();
            // If difference in velocity is larger than 1m/s, the robot does not have the ball
            if(1 < velDifference)
                continue;

            // The robot has the ball
            weHaveBall = true;
//            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Our robot " << robot.id << " has the ball. distanceRobotToBall=" << distanceRobotToBall << ", velDifference=" << velDifference);
        }

        /// Do both and opponent and us have the ball?  => No change
        /// Does no one have the ball? => No change
        /// Does an opponent have the ball? => Defense
        /// Do we have the ball? => Offense

        if(opponentHasBall ^ weHaveBall){
            if(opponentHasBall) {
                ROS_WARN_STREAM_COND_NAMED(weAreAttacking, ROS_LOG_NAME, "We are now defending!");
                weAreAttacking = false;
            }
            if(weHaveBall) {
                ROS_WARN_STREAM_COND_NAMED(!weAreAttacking, ROS_LOG_NAME, "We are now attacking!");
                weAreAttacking = true;
            }
        }

        // === Set the minimumDangerScore based on attacking or defending === //
        float minimumDangerScore;
        if(weAreAttacking){
            // Set the minimumDangerScore quite high, so that we don't defend a lot and have more attackers
            minimumDangerScore = 8.0;
        }else{
            minimumDangerScore = 3.0;
        }

        // === Find the number of dangerous opponents === //
        // TODO : Implement a threshold that prevents robots from continously switching between dangerous and not-dangerous
        std::vector<int> dangerousOpps = RobotsToDefendFinder::GetRobotsToDefend(minimumDangerScore, false);

        // === If the number of dangerous opponents have changed, reset === //
        double shouldReset = false;
        if(dangerousOpps.size() == prevDangerousOpps.size()){
            // The number of dangerous opponents haven't changed. Check if there are different robots
            for (size_t i = 0; i < dangerousOpps.size(); i++) {
                if (dangerousOpps[i] != prevDangerousOpps[i]){
                    // Robots in the list have changed
                    shouldReset = true;
                }
            }
        }else{
            shouldReset = true;
        }

        // If we need to reinitialize regardless, don't return anything
        if(!forceInitialize)
			// Return if we have to reset or not
			return shouldReset;




        /// ======================== WE HAVE TO RESET, BECAUSE THE NUMBER OF DANGEROUS OPPONENTS CHANGED! ======================== ///
//        ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Resetting! Number of dangerous opponents changed from " << prevDangerousOpps.size() << " to " << dangerousOpps.size());

        // Stringstream used to create a one-line description of all that happens below
        std::stringstream statusString;
        statusString << "Reset | " << prevDangerousOpps.size() << " -> " << dangerousOpps.size() << " | ";

        // Store the dangerous opponents for the next iteration
        prevDangerousOpps = dangerousOpps;

        // First, release all the previously claimed robots
        release_robots(activeRobots);	// Only here from initialize function? activeRobots should always be empty?

        // Empty the vector that holds our claimed robots
        activeRobots.clear();
        // Get all the robots that we could use
        std::vector<int> robots = getAvailableRobots();


        /// First, initialize the keeper ///
        {
            // Get the default roledirective publisher
            auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

            int keeperID = RobotDealer::get_keeper();

            roboteam_msgs::RoleDirective rd;
            rd.robot_id = keeperID;

            bt::Blackboard bb;
            bb.SetInt("ROBOT_ID", keeperID);
            bb.SetInt("KEEPER_ID", keeperID);

            // Create message
            rd.tree = "rtt_jelle/KeeperV2";
            rd.blackboard = bb.toMsg();

            // Add random token and save it for later
            boost::uuids::uuid token = unique_id::fromRandom();
            tokens.push_back(token);
            rd.token = unique_id::toMsg(token);

            // Send to rolenode
            pub.publish(rd);

//            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "    Robot " << keeperID << " is now the keeper");
            statusString << keeperID << "=K ";
        }


        /// Decide how many robots we want as robotDefenders, ballInterceptors, and ballDefenders ///
        int numAvailable = robots.size();
        // Put a robot on each dangerous opponent
        int numRobotDefenders = std::min(numAvailable, (int)dangerousOpps.size());
        numAvailable = robots.size() - numRobotDefenders;
        // Put 2 robots in the wall
        int numBallDefenders = std::min(numAvailable, 2);
        numAvailable = robots.size() - numRobotDefenders - numBallDefenders;


        /// Put the ballDefenders in the wall ///
        if(0 < numBallDefenders){
            // === Get the angle offsets === //
            std::vector<float> angleOffsets;
            float step   = weAreAttacking ? 0.3 : 0.2;    // difference between each robot
            float offset = 0;
            if(1 < numBallDefenders)
                offset = -(numBallDefenders-1) * step/2;
            for(int i = 0; i < numBallDefenders; i++){
                angleOffsets.push_back(offset + i * step);
            }

            // Calculate where the robots have to be positioned, and assign the robots in optimal fashion
            std::vector<Vector2> ballDefendersPositions;
            for (int i = 0; i < numBallDefenders; i++) {
                ballDefendersPositions.push_back(SimpleDefender::computeDefensePoint(world.ball.pos, true, 2.0, angleOffsets.at(i)));
            }
            std::vector<int> ballDefenders = assignRobotsToPositions(robots, ballDefendersPositions, world);

            // Init ballDefenders
            std::vector<Vector2> GoToPos_A_positions;
            GoToPos_A_positions.push_back(Vector2(-4.5, 1.2));
            GoToPos_A_positions.push_back(Vector2(-4.5, -1.2));
            ROS_ERROR_STREAM_COND_NAMED(2 < numBallDefenders, ROS_LOG_NAME, "More ballDefenders than GoToPos_A_positions");

            for(int i = 0; i < numBallDefenders; i++){
                // Remove ID from vectors
                int robotID = ballDefenders.at(i);

                delete_from_vector(robots, robotID);
                claim_robot(robotID);
                boost::uuids::uuid token = init_ballDefender(robotID, angleOffsets.at(i), GoToPos_A_positions.at(i));
                activeRobots.push_back(robotID);
                tokens.push_back(token);

//                ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "    Robot " << robotID << " defends the ball");
                statusString << robotID << "=B ";
            }
        }


        /// Figure out which robotDefender should defend which opponent, and initialize them ///
        if(0 < dangerousOpps.size()) {

            // Get the positions of all robots to defend
            ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Get the positions of all robots to defend");
            std::vector<Vector2> oppPositions;
            for (int oppId : dangerousOpps) {
                ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "    Opponent with id " << oppId << ", vector length : " << (int)world.them.size());
                // Get the position of the robot
                boost::optional<roboteam_msgs::WorldRobot> bot = getWorldBot(oppId, false, world);
                if(!bot) {
                    ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Trying to defend bot that doesn't exist! oppID=" << oppId);
                    continue;
                }
                Vector2 oppPos(bot->pos);
                // Store the position in the array
                oppPositions.push_back(oppPos);
            }

            // Map the positions of the robots to defend to our robots
            ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Map the positions of the robots to defend to our robots");
            std::vector<int> robotsToTheirRobots = Anouk_MultipleDefendersPlay::assignRobotsToPositions(robots, oppPositions, world);

            // For each robot to defend
            ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "For each robot to defend");
            for (int i = 0; i < (int)robotsToTheirRobots.size(); i++) {

                // Get our robot that should defend it
                int robotID = robotsToTheirRobots.at(i);

                // Remove ID from vectors
                delete_from_vector(robots, robotID);
                claim_robot(robotID);
                boost::uuids::uuid token = init_robotDefender(robotID, dangerousOpps.at(i));
                activeRobots.push_back(robotID);
                tokens.push_back(token);

//                ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "    Robot " << robotID << " defends opponent " << dangerousOpps.at(i));
                statusString << robotID << "=" << dangerousOpps.at(i) << " ";
            }
        }


//        ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, (int)getAvailableRobots().size() << " robots left for the offense");
        statusString << " | " << (int)getAvailableRobots().size() << " left over";
        ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, statusString.str());

        return false;





        /** ======== FIND ALL DANGEROUS OPPONENTS, ADD TO dangerousOpps LIST ========
         * All robots in the dangerousOpps list will be assigned a robotDefender (unless there arent enough robots)
         * If a robot is in a position to shoot at the goal, create a robotwall (instead of assigning it a robotdefender)
         */
        /*
        std::vector<roboteam_msgs::WorldRobot> dangerousOpps;
        // For each dangerous robot in the world
        for (size_t i = 0; i < world.dangerList.size(); i++) {
            // If robot is considered dangerous enough
            if (world.dangerScores.at(i) >= minDangerScore) {
                // Get the robot from World
                roboteam_msgs::WorldRobot opp = world.dangerList.at(i);
                // Get the angle between goal<->ball and goal<->robot
                double angleDiffBall = fabs(cleanAngle((Vector2(opp.pos) - ourGoalPos).angle() - (ballPos - ourGoalPos).angle()));
                // If the ball is kinda between the robot and the goal, it might shoot at the goal! Add ball defenders!
                if (angleDiffBall <= 0.15) {
//                    ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "setting minBallDefenders=2, for opponent robot=" << opp.id);
                    minBallDefenders = 2;
                } else {
                    // Check if the robot should be assigned a RobotDefender. If the robot is behind another robot that is already defended, we don't need to assign one.
                    bool addDangerousOpp = true;
                    // For each robot already evaluated and considered dangerous
                    for (size_t j = 0; j < dangerousOpps.size(); j++) {
                        // Get the angle between goal<->dangerousRobot and goal<->robot
                        double angleDiffRobot = fabs(cleanAngle((Vector2(opp.pos) - ourGoalPos).angle() - (Vector2(dangerousOpps.at(j).pos) - ourGoalPos).angle()));
                        // If the robot is on the same line to the goal as dangerousRobot, it is already defended by dangerousRobot its defender.
                        if (angleDiffRobot <= 0.15) {
//                            ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Not adding robotDefender for robot=" << opp.id << ". " << "Already covered by the defender or robot=" << dangerousOpps.at(j).id);
                            addDangerousOpp = false;
                            break;
                        }
                    }
                    // If the robot is not covered yet, add it to the list of dangerous opponents.
                    if (addDangerousOpp) {
//                        ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Adding robotDefender for robot=" << opp.id);
                        dangerousOpps.push_back(opp);
                    }
                }
            }
        }
         */
        /* ========================================================================= */


    }



    void Anouk_MultipleDefendersPlay::Initialize() {

//        ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Initializing...");

        prevDangerousOpps.clear();
        activeRobots.clear();

        numBallDefenders = 0;
        numRobotDefenders = 0;
        numExtraDefenders = 0;
        distBallToGoalThreshold = 4.0;
        weAreAttackingCounter = 0;
        weWereAttacking = false;
        weAreAttacking = false;

		// Force a reinitialization
        reInitializeWhenNeeded(true);

        return;
    }

    bt::Node::Status Anouk_MultipleDefendersPlay::Update() {

		bool shouldReInitialize = reInitializeWhenNeeded(false);

		// We should reinitialize everything
        if(shouldReInitialize){
			return Status::Failure;
		}

        return Status::Running;
    }

    /**
    * Initializes a robot defender
    * @param robotID The ID of our robot
    * @param opponentID The ID of the robot it should defend
    * @return uuid-token of the role directive
    */
    boost::uuids::uuid Anouk_MultipleDefendersPlay::init_robotDefender(int robotID, int opponentID){

        // Get the default roledirective publisher
        auto &pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
        // Get the Keeper ID, which is needed in all blackboards
        int keeperID = RobotDealer::get_keeper();

        /* Create Blackboard */
        bt::Blackboard bb;
        // Set the robot ID
        bb.SetInt("ROBOT_ID", robotID);
        bb.SetInt("KEEPER_ID", keeperID);

        Vector2 goalPos(LastWorld::get_our_goal_center());

        // Set blackboard variables
        ScopedBB(bb, "SimpleDefender_A")
            .setString("stayOnSide", "ourSide")
            .setBool("avoidDefenseAreas"  , true)
            .setString("targetFromType", "position")
            .setDouble("targetFromTypeX", goalPos.x)
            .setDouble("targetFromTypeY", goalPos.y)
            .setString("targetToType", "object")
            .setString("targetToObj", "them")
            .setInt("targetToRobotId", opponentID)
            .setDouble("distanceFromGoalRatio", 0.9)
        ;

        //----------improvement defenders----------
        bb.SetBool("GetBall_B_aimAwayFromTarget", true);
        bb.SetString("GetBall_B_aimAt", "ourgoal");
        bb.SetBool("GetBall_D_aimAwayFromTarget", true);
        bb.SetString("GetBall_D_aimAt", "ourgoal");
        bb.SetBool("GetBall_A_doNotPlayBackAttacker", true);
        bb.SetBool("GetBall_B_doNotPlayBackAttacker", true);
        bb.SetBool("GetBall_C_doNotPlayBackAttacker", true);
        bb.SetBool("GetBall_D_doNotPlayBackAttacker", true);
        bb.SetBool("GetBall_A_useBallSensor", true);
        bb.SetBool("GetBall_B_useBallSensor", true);
        bb.SetBool("GetBall_C_useBallSensor", true);
        bb.SetBool("GetBall_D_useBallSensor", true);
        //----------improvement defenders----------


        /* Create message */
        roboteam_msgs::RoleDirective rd;
        rd.robot_id = robotID;
        rd.tree = "rtt_jim/DefenderRoleGetBall";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);

        return token;

    }

    /**
    * Initializes a ball defender
    * @param robotID The ID of our robot
    * @param angleOffset the angle under which to defend the ball
    * @return uuid-token of the role directive
    */
    boost::uuids::uuid Anouk_MultipleDefendersPlay::init_ballDefender(int robotID, float angleOffset, Vector2 GoToPos_A_pos){

        /* DefenderRoleGetBall
        ┼TREE───────────────────────
        │ 14 Repeat
        │     194 Priority
        │         110 Sequence
        │             168 IsInDefenseArea_A | ourDefenseArea=true
        │             22 GoToPos_A
        │         110 Sequence
        │             171 IsRobotClosestToBall_A
        │             194 Priority
        │                 110 Sequence
        │                     27 TheyHaveBall_A
        │                     194 Priority
        │                         110 Sequence
        │                             174 CanInterceptBallDuel_A
        │                             9 GetBall_D | passOn=true
        │                         9 GetBall_B
        │                 9 GetBall_C | passToBestAttacker=true passOn=true doNotPlayBackDefender=true
        │         90 MemSequence
        │             88 SimpleDefender_A
        │             9 GetBall_A | passToBestAttacker=true passOn=true doNotPlayBackDefender=true
        */


        // Get the default roledirective publisher
        auto &pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
        // Get the Keeper ID, which is needed in all blackboards
        int keeperID = RobotDealer::get_keeper();


        /* Create Blackboard */
        bt::Blackboard bb;
        // Set the robot ID
        bb.SetInt("ROBOT_ID", robotID);
        bb.SetInt("KEEPER_ID", keeperID);

        // Set GoToPos_A
        bb.SetDouble("GoToPos_A_xGoal", GoToPos_A_pos.x);
        bb.SetDouble("GoToPos_A_yGoal", GoToPos_A_pos.y);
        bb.SetDouble("GoToPos_A_angleGoal", 0.0);

        //----------improvement defenders----------
        // Set GoToPos_B and C
        bb.SetBool("GetBall_B_aimAwayFromTarget", true);
        bb.SetString("GetBall_B_aimAt", "ourgoal");
        bb.SetBool("GetBall_D_aimAwayFromTarget", true);
        bb.SetString("GetBall_D_aimAt", "ourgoal");
        bb.SetBool("GetBall_A_doNotPlayBackAttacker", true);
        bb.SetBool("GetBall_B_doNotPlayBackAttacker", true);
        bb.SetBool("GetBall_C_doNotPlayBackAttacker", true);
        bb.SetBool("GetBall_D_doNotPlayBackAttacker", true);
        bb.SetBool("GetBall_A_useBallSensor", true);
        bb.SetBool("GetBall_B_useBallSensor", true);
        bb.SetBool("GetBall_C_useBallSensor", true);
        bb.SetBool("GetBall_D_useBallSensor", true);
        //----------improvement defenders----------

        /* === SimpleDefender === */
        bb.SetDouble("SimpleDefender_A_distanceFromGoal", 2);
        bb.SetDouble("SimpleDefender_A_angleOffset", angleOffset);


        // Create message
        roboteam_msgs::RoleDirective rd;
        rd.robot_id = robotID;
        rd.tree = "rtt_jim/DefenderRoleGetBall";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);

        return token;
    }

} // rtt
