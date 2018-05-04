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

	Emiel_PrepareKickoffThem::Emiel_PrepareKickoffThem(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}

	void Emiel_PrepareKickoffThem::Initialize(){

		// Check if this tree is used for the correct RefState, PREPARE_KICKOFF_THEM
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		if(refState != RefState::PREPARE_KICKOFF_THEM){
			ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_KICKOFF_THEM");
		}


		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		// All but one robot
		int numRobots = robots.size();
		// Calculate positions for ballDefenders
		std::vector<Vector2> positions = RobotPatternGenerator::Circle(numRobots, M_PI, 4.3, Vector2(0, 0), M_PI, 0, -0.2);

		Emiel_Prepare::prepare(positions);

	}

	bt::Node::Status Emiel_PrepareKickoffThem::Update(){
		return Status::Running;
	}

}