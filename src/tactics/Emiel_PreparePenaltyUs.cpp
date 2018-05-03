#include "roboteam_tactics/tactics/Emiel_PreparePenaltyUs.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Emiel_PreparePenaltyUs"

namespace rtt {

	RTT_REGISTER_TACTIC(Emiel_PreparePenaltyUs);

	Emiel_PreparePenaltyUs::Emiel_PreparePenaltyUs(std::string name, bt::Blackboard::Ptr blackboard)
			: Tactic(name, blackboard) {
		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Creating...");
	}

	void Emiel_PreparePenaltyUs::Initialize(){
		ROS_INFO_NAMED(ROS_LOG_NAME, "Initializing");

		// Check if this tree is used for the correct RefState, DO_KICKOFF
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		if(refState != RefState::PREPARE_PENALTY_US){
			ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_PENALTY_US");
		}

		// Get the default roledirective publisher
		auto &pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
		// Get the Keeper ID, which is needed in all blackboards
		int keeperID = RobotDealer::get_keeper();
		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		// Get the world
		roboteam_msgs::World world = LastWorld::get();

		// Clear the tokens
		tokens.clear();

		// =========================
		// Initialize the Keeper
		// =========================
		{
			claim_robot(keeperID);

			// Create blackboard
			bt::Blackboard bb;
			bb.SetInt("ROBOT_ID", keeperID);
			bb.SetInt("KEEPER_ID", keeperID);

			// Create message
			roboteam_msgs::RoleDirective rd;
			rd.robot_id = keeperID;
			rd.tree = "rtt_jim/DefenderRoleStop";
			rd.blackboard = bb.toMsg();

			// Add random token and save it for later
			boost::uuids::uuid token = unique_id::fromRandom();
			tokens.push_back(token);
			rd.token = unique_id::toMsg(token);

			// Send to rolenode
			pub.publish(rd);
		}

		// =========================
		// Initialize the Kickoff-taker
		// =========================
		{
			// Claim a robot to be the kicker
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
			bb.SetDouble("GoToPos_A_angleGoal", 0);
			bb.SetDouble("GoToPos_A_xGoal", 4.8);
			bb.SetDouble("GoToPos_A_yGoal", 0);
			bb.SetBool("GoToPos_A_avoidRobots", true);
			bb.SetDouble("GoToPos_A_maxSpeed", 1.3);

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


		// ====================================
		// Initialize the Ball Defenders!
		// ====================================
		int numBallDefenders = robots.size();

		// Calculate positions for ballDefenders
		std::tuple<std::vector<float>, std::vector<float>> anglesAndDistances;
		anglesAndDistances = RobotPatternGenerator::Line(numBallDefenders, 5, LastWorld::get_our_goal_center(), 0, 8);
		std::vector<float> angleOffsets = std::get<0>(anglesAndDistances);
		std::vector<float> distanceOffsets = std::get<1>(anglesAndDistances);

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

			bb.SetDouble("SimpleDefender_A_maxSpeed", 1.3);

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
	}

	bt::Node::Status Emiel_PreparePenaltyUs::Update(){
		return Status::Running;
	}

}