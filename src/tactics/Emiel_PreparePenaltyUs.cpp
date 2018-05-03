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

		// Check if this tree is used for the correct RefState, PREPARE_PENALTY_US
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
			// Get its ID
			int kickerID = robots.at(0);
			// Remove ID from vectors
			delete_from_vector(robots, kickerID);
			claim_robot(kickerID);

			/* Create Blackboard */
			bt::Blackboard bb;
			// Set the robot ID
			bb.SetInt("ROBOT_ID", kickerID);
			bb.SetInt("KEEPER_ID", keeperID);
			// Set positioning variables
			ScopedBB(bb, "GoToPos_A")
				.setDouble("angleGoal"  , 0)
				.setDouble("xGoal"      , 4.8)
				.setDouble("yGoal"      , 0)
				.setBool  ("avoidRobots", true)
				.setDouble("maxSpeed"   , 1.3);

			/* Create message */
			roboteam_msgs::RoleDirective rd;
			rd.robot_id = kickerID;
			rd.tree = "rtt_jim/GoToPosRole";
			rd.blackboard = bb.toMsg();

			// Add random token and save it for later
			boost::uuids::uuid token = unique_id::fromRandom();
			tokens.push_back(token);
			rd.token = unique_id::toMsg(token);

			// Send to rolenode
			pub.publish(rd);
		}



		// ================================
		// Initialize Defenders / Attackers
		// ================================

		// Number of defenders that should stay behind
		int numBallDefenders = std::min((int)robots.size(), 2);
		// Number of attackers that should go to the front
		int numAttackers = std::max((int)robots.size()-numBallDefenders, 0);

		// Calculate positions for defenders and attackers
		/* Creates a line of 2? robots, 1 meter wide, 2 meters in front of our goal center */
		std::vector<Vector2> defenderCoords = RobotPatternGenerator::Line(numBallDefenders, 1, LastWorld::get_our_goal_center(), 0, 2);
		/* Creates a circle of 5? robots, covering M_PI/2 rad, in a circle with radius 3, around their goal center, with a M_PI rad phase shift */
		std::vector<Vector2> attackerCoords = RobotPatternGenerator::Circle(numAttackers, M_PI/2, 3, LastWorld::get_their_goal_center(), M_PI, 0, 0);

		// Put all positions into one array
		std::vector<Vector2> x_y_coords;
		x_y_coords.insert(std::end(x_y_coords), std::begin(defenderCoords), std::end(defenderCoords));
		x_y_coords.insert(std::end(x_y_coords), std::begin(attackerCoords), std::end(attackerCoords));

		// Calculate which robot should to go which position
		std::vector<int> robotIDs = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, x_y_coords, world);

		for (size_t i = 0; i < robotIDs.size(); i++) {
			// Get its ID
			int robotID = robotIDs.at(i);
			// Remove ID from vectors
			delete_from_vector(robots, robotID);
			claim_robot(robotID);

			/* Create Blackboard */
			bt::Blackboard bb;
			// Set the robot ID
			bb.SetInt("ROBOT_ID", robotID);
			bb.SetInt("KEEPER_ID", keeperID);
			// Set positioning variables
			ScopedBB(bb, "GoToPos_A")
				.setDouble("angleGoal"  , 0)
				.setDouble("xGoal"      , x_y_coords.at(i).x)
				.setDouble("yGoal"      , x_y_coords.at(i).y)
				.setBool  ("avoidRobots", true)
				.setDouble("maxSpeed"   , 1.3);

			/* Create message */
			roboteam_msgs::RoleDirective rd;
			rd.robot_id = robotID;
			rd.tree = "rtt_jim/GoToPosRole";
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