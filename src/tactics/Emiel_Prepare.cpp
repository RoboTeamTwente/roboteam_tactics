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

	std::vector<boost::uuids::uuid> Emiel_Prepare::prepare(RefState refState, std::vector<Vector2> positions){
		return prepare(refState, positions, std::vector<int>(), std::vector<int>());
	}

    std::vector<boost::uuids::uuid> Emiel_Prepare::prepare(RefState refState, std::vector<Vector2> positions, std::vector<int> robotsToDefend){
        return prepare(refState, positions, robotsToDefend, std::vector<int>());
    }

	std::vector<boost::uuids::uuid> Emiel_Prepare::prepare(RefState refState, std::vector<Vector2> positions, std::vector<int> robotsToDefend, std::vector<int> robotsToIntercept) {

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
			ROS_WARN_STREAM_NAMED("Emiel_Prepare", "Not enough robots to fill positions! : #positions: " << positions.size() + ", robots : " << robots.size());
		}
		if (robots.size() - positions.size() < robotsToDefend.size()) {
			ROS_WARN_STREAM_NAMED("Emiel_Prepare", "Not enough robots to defend opponents! : #opponents: " << robotsToDefend.size() + ", robots : " << (robots.size() - positions.size()));
		}

		// ============================
		///Initialize the normal Keeper
		// ============================
		{
			claim_robot(keeperID);

			// Create blackboard
			bt::Blackboard bb;
			bb.SetInt("ROBOT_ID", keeperID);
			bb.SetInt("KEEPER_ID", keeperID);

			// Create message
			roboteam_msgs::RoleDirective rd;
			rd.robot_id = keeperID;

			if(refState == RefState::PREPARE_PENALTY_THEM || refState == RefState::DEFEND_PENALTY)
				rd.tree = "rtt_anouk/PenaltyKeeperRole";
			else
				rd.tree = "rtt_jim/DefenderRoleStop";

			rd.blackboard = bb.toMsg();

			// Add random token and save it for later
			boost::uuids::uuid token = unique_id::fromRandom();
			tokens.push_back(token);
			rd.token = unique_id::toMsg(token);

			// Send to rolenode
			pub.publish(rd);
		}

		// =====================================
		///Initialize positions / ball defenders
		// =====================================
		// Calculate which robot should take which position
		std::vector<int> robotsToPositions = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, positions, world);
		// For each robot
//		ROS_INFO_STREAM_NAMED("Emiel_Prepare", "Mapping our robots to positions to defend...");
		for (size_t i = 0; i < robotsToPositions.size(); i++) {
			// Get its ID
			int robotID = robotsToPositions.at(i);
			// Remove ID from vectors
			delete_from_vector(robots, robotID);
			claim_robot(robotID);
			// Initialize the ball defender
			boost::uuids::uuid token = init_ballDefender(robotID, positions.at(i));
			tokens.push_back(token);
//			ROS_INFO_STREAM_NAMED("Emiel_Prepare", "Claimed robot " << robotID << " for position " << positions.at(i));
		}



		// ====================================
		///Initialize robot defenders
		// ====================================
		if(0 < robotsToDefend.size()) {
			ROS_INFO_STREAM_NAMED("Emiel_Prepare", "Mapping our robots to their robots...");

			// Get the positions of all robots to defend
			std::vector<Vector2> oppPositions;
			for (int oppId : robotsToDefend) {
				// Get the position of the robot
				Vector2 oppPos(world.them.at(oppId).pos);
				// Store the position in the array
				oppPositions.push_back(oppPos);
			}

			// Map the positions of the robots to defend to our robots
			std::vector<int> robotsToTheirRobots = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, oppPositions, world);

			// For each robot to defend
			for (int i = 0; i < (int)robotsToTheirRobots.size(); i++) {

				// ====================================
				// Initialize simple defender
				// ====================================

				// Get our robot that should defend it
				int robotID = robotsToTheirRobots.at(i);

				// Remove ID from vectors
				delete_from_vector(robots, robotID);
				claim_robot(robotID);

				boost::uuids::uuid token = init_robotDefender(robotID, robotsToDefend.at(i));
				tokens.push_back(token);
				ROS_INFO_STREAM_NAMED("Emiel_Prepare", "Robot " << robotID << " defends opponent " << robotsToDefend.at(i));
			}
		}


		// ====================================
		///Initialize intercept defenders
		// ====================================
		if(0 < robots.size()) {
			ROS_INFO_STREAM_NAMED("Emiel_Prepare", "Mapping our robots to their robots...");

			// Get the positions of all robots to defend
			std::vector<Vector2> oppPositionsInt;
			for (int oppIdInt : robotsToIntercept) {
				// Get the position of the robot
				Vector2 oppPosInt(world.them.at(oppIdInt).pos);
				// Store the position in the array
				oppPositionsInt.push_back(oppPosInt);
			}

			// Map the positions of the robots to defend to our robots
			std::vector<int> robotsToTheirRobotsInt = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, oppPositionsInt, world);

			// For each robot to defend
			for (int i = 0; i < (int)robotsToTheirRobotsInt.size(); i++) {

				// ====================================
				// Initialize simple defender
				// ====================================

				// Get our robot that should defend it
				int robotIDInt = robotsToTheirRobotsInt.at(i);

				// Remove ID from vectors
				delete_from_vector(robots, robotIDInt);
				claim_robot(robotIDInt);

				boost::uuids::uuid token = init_interceptDefender(robotIDInt, robotsToIntercept.at(i));
				tokens.push_back(token);
				ROS_INFO_STREAM_NAMED("Emiel_Prepare", "Robot " << robotIDInt << " intercepts opponent " << robotsToIntercept.at(i));
			}
		}

		return tokens;
	}

	/**
	 * Initializes a ball defender
	 * @param robotID The ID of our robot
	 * @param position The position the robot should defend
	 * @return uuid-token of the role directive
	 */
	boost::uuids::uuid Emiel_Prepare::init_ballDefender(int robotID, Vector2 position){

		// Get the default roledirective publisher
		auto &pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
		// Get the Keeper ID, which is needed in all blackboards
		int keeperID = RobotDealer::get_keeper();

		/* Create Blackboard */
		bt::Blackboard bb;
		// Set the robot ID
		bb.SetInt("ROBOT_ID", robotID);
		bb.SetInt("KEEPER_ID", keeperID);

		// Get the position of the ball
		roboteam_msgs::World world = LastWorld::get();
		Vector2 ballPos;
		if (!std::isnan(world.ball.pos.x)) {
			ballPos = world.ball.pos;
		}

		// Calculate angle to ball
		float angleRobotToBall = (ballPos - position).angle();

		// Set positioning variables
		ScopedBB(bb, "GoToPos_A")
			.setDouble("angleGoal", angleRobotToBall)
			.setDouble("xGoal"    , position.x)
			.setDouble("yGoal"    , position.y)
			.setBool("avoidRobots", true)
			.setBool("avoidBall"  , true)
			.setBool("stayAwayFromBall"  , true)
			.setBool("avoidDefenseAreas"  , true)
			.setDouble("maxSpeed" , 1.3)
		;

		/* Create message */
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = robotID;
		rd.tree = "rtt_emiel/GoToPos_Nonstop_Role";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		rd.token = unique_id::toMsg(token);

		// Send to rolenode
		pub.publish(rd);

		return token;
	}

	/**
	 * Initializes a robot defender
	 * @param robotID The ID of our robot
	 * @param opponentID The ID of the robot it should defend
	 * @return uuid-token of the role directive
	 */
	boost::uuids::uuid Emiel_Prepare::init_robotDefender(int robotID, int opponentID){

		// Get the default roledirective publisher
		auto &pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
		// Get the Keeper ID, which is needed in all blackboards
		int keeperID = RobotDealer::get_keeper();

		/* Create Blackboard */
		bt::Blackboard bb;
		// Set the robot ID
		bb.SetInt("ROBOT_ID", robotID);
		bb.SetInt("KEEPER_ID", keeperID);

		// Set blackboard variables
		ScopedBB(bb, "SimpleDefender_A")
			.setString("stayOnSide", "ourSide")
			.setBool("avoidDefenseAreas"  , true)
			.setBool("stayAwayFromBall", true)
			.setBool("avoidBall"  , true)
			.setBool("dontDriveToBall", true)
			.setDouble("A_maxSpeed", 1.3)
			.setString("targetFromType", "position")
			.setDouble("targetFromTypeX", -6.0)
			.setDouble("targetFromTypeY", 0.0)
			.setString("targetToType", "object")
			.setString("targetToObj", "them")
			.setInt("targetToRobotId", opponentID)
			.setDouble("distanceFromGoalRatio", 0.9)
		;

		/* Create message */
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = robotID;
		rd.tree = "rtt_jim/DefenderRoleStop";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		rd.token = unique_id::toMsg(token);

		// Send to rolenode
		pub.publish(rd);

		return token;

	}

	/**
 	* Initializes a intercept defender
 	* @param robotID The ID of our robot
 	* @param opponentID The ID of the robot it should intercept
	* @return uuid-token of the role directive
	*/
	boost::uuids::uuid Emiel_Prepare::init_interceptDefender(int robotID, int opponentInterceptID){

		// Get the default roledirective publisher
		auto &pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
		// Get the Keeper ID, which is needed in all blackboards
		int keeperID = RobotDealer::get_keeper();

		/* Create Blackboard */
		bt::Blackboard bb;
		// Set the robot ID
		bb.SetInt("ROBOT_ID", robotID);
		bb.SetInt("KEEPER_ID", keeperID);

		// Set blackboard variables
		ScopedBB(bb, "SimpleDefender_A")
			.setBool("stayAwayFromBall", true)
			.setBool("avoidBall", true)
			.setBool("dontDriveToBall", true)
			.setDouble("A_maxSpeed", 1.3)
			.setString("targetToType", "object")
			.setString("targetToObj", "ball")
			.setString("targetFromType", "object")
			.setString("targetFromObj", "them")
			.setInt("targetFromRobotId", opponentInterceptID)
			.setDouble("distanceFromGoalRatio", 0.5)
			;

		/* Create message */
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = robotID;
		rd.tree = "rtt_jim/DefenderRoleStop";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		rd.token = unique_id::toMsg(token);

		// Send to rolenode
		pub.publish(rd);

		return token;

	}


}