#include "roboteam_tactics/tactics/Emiel_PrepareKickoffThem.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Emiel_PrepareKickoffThem"

namespace rtt {

	RTT_REGISTER_TACTIC(Emiel_PrepareKickoffThem);

	Emiel_PrepareKickoffThem::Emiel_PrepareKickoffThem(std::string name, bt::Blackboard::Ptr blackboard)
			: Tactic(name, blackboard) {
		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Creating...");
	}

	void Emiel_PrepareKickoffThem::Initialize(){
		ROS_INFO_NAMED(ROS_LOG_NAME, "Initializing");

		// Check if this tree is used for the correct RefState, PREPARE_KICKOFF_THEM
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		if(refState != RefState::PREPARE_KICKOFF_THEM){
			ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_KICKOFF_THEM");
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



		// ====================================
		// Initialize the Ball Defenders!
		// ====================================

		static std::vector<Vector2> x_y_coords;
		int numBallDefenders;
		std::vector<int> ballDefenders;

		// Calculate the number of robots to use
		numBallDefenders = robots.size();
		// Calculate positions for ballDefenders
		x_y_coords = RobotPatternGenerator::Circle(numBallDefenders, M_PI, 4.3, Vector2(0, 0), M_PI, 0, -0.2);
		// Calculate which robot should take which position
		ballDefenders = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, x_y_coords, world);

		// For each robot
		for (size_t i = 0; i < ballDefenders.size(); i++) {
			// Get its ID
			int ballDefenderID = ballDefenders.at(i);
			// Remove ID from vectors
			delete_from_vector(robots, ballDefenderID);
			claim_robot(ballDefenderID);

			/* Create Blackboard */
			bt::Blackboard bb;
			// Set the robot ID
			bb.SetInt("ROBOT_ID", ballDefenderID);
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
			rd.robot_id = ballDefenderID;
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

	bt::Node::Status Emiel_PrepareKickoffThem::Update(){
		return Status::Running;
	}

}