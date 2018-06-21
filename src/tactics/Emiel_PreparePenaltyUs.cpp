#include "roboteam_tactics/tactics/Emiel_PreparePenaltyUs.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"


#include "roboteam_tactics/utils/RobotPatternGenerator.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Emiel_PreparePenaltyUs"

namespace rtt {

	RTT_REGISTER_TACTIC(Emiel_PreparePenaltyUs);

	Emiel_PreparePenaltyUs::Emiel_PreparePenaltyUs(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}

	void Emiel_PreparePenaltyUs::Initialize(){

		// Get the latest world
		const roboteam_msgs::World& world = LastWorld::get();

		// Check if this tree is used for the correct RefState, PREPARE_PENALTY_US
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		if(refState != RefState::PREPARE_PENALTY_US){
			ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_PENALTY_US");
		}

		// Check if ball is our side, which would mean shootout
		bool inShootout = false;
		if(world.ball.pos.x < 0){
			ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "PENALTY SHOOTOUT DETECTED! Initializing defense");
			inShootout = true;
//			Emiel_PreparePenaltyUs::initShootout();
//			return;
		}

		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		// Holds positions
		std::vector<Vector2> positions;
		// Add kicker
		Vector2 kickerPos = (Vector2)LastWorld::get().ball.pos - Vector2(0.5, 0);
		positions.push_back(kickerPos);

		// All but one robot
		int numRobots = robots.size() - 1;
		// Number of defenders that should stay behind
		int numDefenders = std::min(numRobots, 2);
		// Number of attackers that should go to the front
		int numAttackers = std::max(numRobots-numDefenders, 0);

		Vector2 defendPos = (Vector2)LastWorld::get_our_goal_center();
		if(inShootout){
			numDefenders = robots.size() - 1;
			numAttackers = 0;
			defendPos = Vector2(-7.8, -3);
		}

		// Calculate positions for defenders and attackers
		/* Creates a line of 2? robots, 1 meter wide, 2 meters in front of our goal center */
		std::vector<Vector2> defenderCoords = RobotPatternGenerator::Line(numDefenders, 1, defendPos, 0, 2);
		/* Creates a line of 5? robots, 7.2? meter wide, 3 meters in front of their goal center */
		float lineLength = LastWorld::get_field().field_width * 0.8;
		std::vector<Vector2> attackerCoords = RobotPatternGenerator::Line(numAttackers, lineLength, LastWorld::get_their_goal_center(), 0, -3);


		// Put all positions into one array
		positions.insert(std::end(positions), std::begin(defenderCoords), std::end(defenderCoords));
		positions.insert(std::end(positions), std::begin(attackerCoords), std::end(attackerCoords));

		Emiel_Prepare::prepare(inShootout ? RefState::DEFEND_PENALTY : *refState, positions);

	}

	bt::Node::Status Emiel_PreparePenaltyUs::Update(){
		return Status::Running;
	}

}