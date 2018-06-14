#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>
#include <tuple>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_KickOffDefense.h"
#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/RefLookup.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"

static const std::string ROS_LOG_NAME = "plays.JimKOD";

namespace rtt {

RTT_REGISTER_TACTIC(Jim_KickOffDefense);


Jim_KickOffDefense::Jim_KickOffDefense(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard)
        , weHaveBall("", blackboard)
        {}


void Jim_KickOffDefense::Initialize() {
	tokens.clear();

	ROS_INFO_NAMED("plays.JimKOD", "Initializing Jim_KickOffDefense");
	if (getAvailableRobots().size() < 1) {
		ROS_WARN_NAMED("plays.JimKOD", "Not enough robots, cannot initialize..");
		// TODO: Want to pass failure here as well!
		return;
	}

	boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();

	if(refState == RefState::PREPARE_KICKOFF_US){
		ROS_INFO_NAMED(ROS_LOG_NAME, "Preparing kickoff for us");
	}
	if(refState == RefState::PREPARE_KICKOFF_THEM){
		ROS_INFO_NAMED(ROS_LOG_NAME, "Preparing kickoff for them");
	}

	bool kickoffForUs = getExtendedState() == RefState::DO_KICKOFF;

	// Get the world
	roboteam_msgs::World world = LastWorld::get();
	Vector2 ourGoalPos = LastWorld::get_our_goal_center();
//	Vector2 ballPos = world.ball.pos;
	std::vector<roboteam_msgs::WorldRobot> theirRobots = world.them;


	std::vector<int> robots = getAvailableRobots();
	int keeperID = RobotDealer::get_keeper();
	// Get the default roledirective publisher
	auto &pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();


	// =========================
	// Initialize the Keeper
	// =========================
	{
		claim_robot(keeperID);

		roboteam_msgs::RoleDirective rd;
		rd.robot_id = keeperID;
		bt::Blackboard bb;

		bb.SetInt("ROBOT_ID", keeperID);
		bb.SetInt("KEEPER_ID", keeperID);

		// Create message
		rd.tree = "rtt_jim/DefenderRoleStop";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		tokens.push_back(token);
		rd.token = unique_id::toMsg(token);

		// Send to rolenode
		pub.publish(rd);
	}


	int availableRobots = getAvailableRobots().size();

	// =========================
	// Initialize the Kickoff-taker if needed
	// =========================

	ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "kickoffForUs=" << (kickoffForUs ? "True" : "False"));
	ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "refState=" << refStateToString(*refState).c_str());

	if(kickoffForUs){
		int kickerID = robots.at(0);
		delete_from_vector(robots, kickerID);
		claim_robot(kickerID);

		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Claiming robot " << kickerID << " as kicker");

		roboteam_msgs::RoleDirective rd;
		rd.robot_id = kickerID;
		bt::Blackboard bb;

		bb.SetInt("ROBOT_ID", kickerID);
		bb.SetInt("KEEPER_ID", keeperID);

		// Set positioning variables
		bb.SetDouble("GoToPos_A_angleGoal", M_PI/2);
		bb.SetDouble("GoToPos_A_xGoal", 0);
		bb.SetDouble("GoToPos_A_yGoal", -0.5);
		bb.SetBool("GoToPos_A_avoidRobots", true);

		if (HasBool("driveSlow") && GetBool("driveSlow")) {
			bb.SetDouble("GoToPos_A_maxSpeed", 1.3);
		}

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


	// Get two or less robot defenders
	int numRobotDefenders = std::min((int) getAvailableRobots().size(), 2);

	// ==================================
	// Find the most dangerous opponents
	// ==================================
	std::vector<roboteam_msgs::WorldRobot> dangerousOpps;
	for (size_t i = 0; i < world.dangerList.size(); i++) {
		roboteam_msgs::WorldRobot opp = world.dangerList.at(i);
//		double angleDiffBall = fabs(cleanAngle((Vector2(opp.pos) - ourGoalPos).angle() - (ballPos - ourGoalPos).angle()));
		{

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

	ROS_DEBUG_NAMED("plays.JimKOD", "num KickOffDefenders: %i", numRobotDefenders);

	for (int i = 0; i < numRobotDefenders; i++) {

		roboteam_msgs::WorldRobot mostDangerousRobot = dangerousOpps.at(i);
		int defenderID = defenderIDs.at(i);

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
		if (HasBool("driveSlow") && GetBool("driveSlow")) {
			bb.SetDouble("SimpleDefender_A_maxSpeed", 1.3);
		}

		// Create message
		rd.tree = "rtt_jim/DefenderRoleStop";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		tokens.push_back(token);
		rd.token = unique_id::toMsg(token);


		// Send to rolenode
		pub.publish(rd);
	}


	int numBallDefenders;
	std::vector<float> angleOffsets;
	std::vector<float> distanceOffsets;
	availableRobots = getAvailableRobots().size();

	numBallDefenders = availableRobots;


	// === Creates a straight line of robots, 4 meters in width, 2 meters in front of the goal === //
	{
//		float lineWidth = 4.0;
//		float goalDistance = 2.0;
//
//		float step = lineWidth / (numBallDefenders - 1);  // y-distance between each robot. Gives a line 4m wide
//		float offset = -(numBallDefenders - 1) * step / 2;  // step=4 -> 4=-6, 3=-4, 2=-2, 1=0
//
//		for (int i = 0; i < numBallDefenders; i++) {
//
//			float y = offset + i * step;
//			float x = sqrt(pow(goalDistance, 2) + pow(y, 2)) + /*gives the nice curve->*/ abs(offset + i * step) / 2;
//			float w = atan(y / goalDistance);
//
//			angleOffsets.push_back(w);
//			distanceOffsets.push_back(x);
//		}
	}
	// ======================================================================== //

	// === Creates an overly complicated curve because it looks cool === //
	{
		float angleTotal = M_PI * 0.8;
		float circleRadius = 1.5;
		float circleFromGoal = 4.5;

		float angleStep = 0;
		if(numBallDefenders > 1)
			angleStep = angleTotal / (numBallDefenders - 1);

		float angleOffset = -(numBallDefenders - 1) * angleStep / 2;

		for (int i = 0; i < numBallDefenders; i++) {

			float A = angleOffset + i * angleStep;

			float b = circleRadius + circleFromGoal;
			float c = circleRadius;

			float b2 = pow(b, 2);
			float c2 = pow(c, 2);

			float a2 = b2 + c2 - 2 * b * c * cos(A);
			float a = sqrt(a2);

			float C = acos( (a2 + b2 - c2) / ( 2 * a * b ));

			if(A < 0)
				C = -C;

			angleOffsets.push_back(C);
			distanceOffsets.push_back(a);
		}

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
        bb.SetBool("SimpleDefender_A_avoidRobots", true);
        bb.SetBool("SimpleDefender_A_dontDriveToBall", true);
        bb.SetBool("SimpleDefender_A_stayAwayFromBall", true);
		bb.SetBool("SimpleDefender_A_avoidBall", true);

        if (HasBool("driveSlow") && GetBool("driveSlow")) {
            bb.SetDouble("SimpleDefender_A_maxSpeed", 1.3);
        }

        // Create message
        rd.tree = "rtt_jim/DefenderRoleStop";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }


    initTime = now();
    
}


bt::Node::Status Jim_KickOffDefense::Update() {

    // Re-initialize tree every second. Needed to check for new dangerous opponents
//    if (time_difference_milliseconds(initTime, now()).count() >= 1000) {
//        return Status::Failure;
//    }

    roboteam_msgs::World world = LastWorld::get();
    Vector2 ballVel(world.ball.vel);

    // Check if ball has moved
    if (ballVel.length() > 0.5 && (HasBool("allowSuccess") && GetBool("allowSuccess"))) {
        return Status::Success;
    }

    return Status::Running;
}

} // rtt
