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

	Emiel_PreparePenaltyUs::Emiel_PreparePenaltyUs(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}

	void Emiel_PreparePenaltyUs::Initialize(){

		// Check if this tree is used for the correct RefState, PREPARE_PENALTY_US
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		if(refState != RefState::PREPARE_PENALTY_US){
			ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_PENALTY_US");
		}

		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		// Holds positions
		std::vector<Vector2> positions;
		// Add kicker
		positions.push_back(Vector2(4.3, 0));

		// All but one robot
		int numRobots = robots.size() - 1;
		// Number of defenders that should stay behind
		int numDefenders = std::min(numRobots, 2);
		// Number of attackers that should go to the front
		int numAttackers = std::max(numRobots-numDefenders, 0);

		// Calculate positions for defenders and attackers
		/* Creates a line of 2? robots, 1 meter wide, 2 meters in front of our goal center */
		std::vector<Vector2> defenderCoords = RobotPatternGenerator::Line(numDefenders, 1, LastWorld::get_our_goal_center(), 0, 2);
		/* Creates a circle of 5? robots, covering M_PI/2 rad, in a circle with radius 3, around their goal center, with a M_PI rad phase shift */
		std::vector<Vector2> attackerCoords = RobotPatternGenerator::Circle(numAttackers, M_PI/2, 3, LastWorld::get_their_goal_center(), M_PI, 0, 0);

		// Put all positions into one array
		positions.insert(std::end(positions), std::begin(defenderCoords), std::end(defenderCoords));
		positions.insert(std::end(positions), std::begin(attackerCoords), std::end(attackerCoords));

		Emiel_Prepare::prepare(*refState, positions);

	}

	bt::Node::Status Emiel_PreparePenaltyUs::Update(){
		return Status::Running;
	}

}