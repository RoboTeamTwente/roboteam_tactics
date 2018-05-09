#include <tuple>
#include <vector>
#include <ros/ros.h>

#include "roboteam_tactics/tactics/Emiel_Prepare.h"
#include "roboteam_tactics/utils/RobotPatternGenerator.h"
#include "roboteam_tactics/Parts.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

namespace rtt {

	Emiel_Prepare::Emiel_Prepare(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard){}

	std::vector<boost::uuids::uuid> Emiel_Prepare::prepare(std::vector<Vector2> positions) {

		std::vector<boost::uuids::uuid> tokens;

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

		if (robots.size() < positions.size()) {
			ROS_WARN_STREAM_NAMED("Emiel_Prepare", "More positions assigned than robots available! positions : " << positions.size() + ", robots : " << robots.size());
		}

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
		// Initialize positions
		// ====================================

		// Calculate which robot should take which position
		std::vector<int> robotsToPositions = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, positions, world);

		// For each robot
		for (size_t i = 0; i < robotsToPositions.size(); i++) {
			// Get its ID
			int robotID = robotsToPositions.at(i);
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
				.setDouble("angleGoal", 0)
				.setDouble("xGoal"    , positions.at(i).x)
				.setDouble("yGoal"    , positions.at(i).y)
				.setBool("avoidRobots", true)
				.setBool("avoidBall"  , true)
				.setDouble("maxSpeed" , 1.3);

			/* Create message */
			roboteam_msgs::RoleDirective rd;
			rd.robot_id = robotID;
			rd.tree = "rtt_emiel/GoToPos_Nonstop_Role";
			rd.blackboard = bb.toMsg();

			// Add random token and save it for later
			boost::uuids::uuid token = unique_id::fromRandom();
			tokens.push_back(token);
			rd.token = unique_id::toMsg(token);

			// Send to rolenode
			pub.publish(rd);
		}

		return tokens;
	}

}