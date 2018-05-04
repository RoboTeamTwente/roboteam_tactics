#include "roboteam_tactics/tactics/Emiel_PreparePenaltyThem.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Emiel_PreparePenaltyThem"

namespace rtt {

	RTT_REGISTER_TACTIC(Emiel_PreparePenaltyThem);

	Emiel_PreparePenaltyThem::Emiel_PreparePenaltyThem(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}

	void Emiel_PreparePenaltyThem::Initialize(){

		// Check if this tree is used for the correct RefState, PREPARE_PENALTY_THEM
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		if(refState != RefState::PREPARE_PENALTY_THEM){
			ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_PENALTY_THEM");
		}

		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		// Number of defenders that should stay behind
		int numDefenders = std::min((int)robots.size(), 5);
		// Number of attackers that should go to the front
		int numAttackers = std::max((int)robots.size()-numDefenders, 0);

		// Calculate positions for defenders and attackers
		/* Creates a line of 5? robots, 1.2 meter wide, 2 meters in front of our goal */
		std::vector<Vector2> defenderCoords = RobotPatternGenerator::Line(numDefenders, 2, Vector2(-4.8, 0), 0, 0);
		/* Creates a line of 2? robots, covering M_PI/2 rad, in a circle with radius 3, around their goal center, with a M_PI rad phase shift */
		std::vector<Vector2> attackerCoords = RobotPatternGenerator::Line(numAttackers, 6, Vector2(3, 0), 0, 0);

		// Put all positions into one array
		std::vector<Vector2> positions;
		positions.insert(std::end(positions), std::begin(defenderCoords), std::end(defenderCoords));
		positions.insert(std::end(positions), std::begin(attackerCoords), std::end(attackerCoords));

		Emiel_Prepare::prepare(positions);

	}

	bt::Node::Status Emiel_PreparePenaltyThem::Update(){
		return Status::Running;
	}

}