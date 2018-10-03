#include "roboteam_tactics/tactics/Emiel_PrepareKickoffUs.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Emiel_PrepareKickoffUs"

namespace rtt {

	RTT_REGISTER_TACTIC(Emiel_PrepareKickoffUs);

	Emiel_PrepareKickoffUs::Emiel_PrepareKickoffUs(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}

	void Emiel_PrepareKickoffUs::Initialize(){

		// Check if this tree is used for the correct RefState, PREPARE_KICKOFF_US
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		if(refState != RefState::PREPARE_KICKOFF_US){
			ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_KICKOFF_US");
		}

		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		// All but one robot
		int numRobots = robots.size() - 1;
        // == Calculate positions for ballDefenders == //
        // Get the radius of the circle
        float radius = LastWorld::get_field().field_width * 0.45;
		std::vector<Vector2> positions = RobotPatternGenerator::Circle(numRobots, M_PI, radius, Vector2(0, 0), M_PI, 0, -0.2);
		// Add the kicker position
		positions.push_back(Vector2(-0.2, 0));

		Emiel_Prepare::prepare(*refState, positions);

	}

	bt::Node::Status Emiel_PrepareKickoffUs::Update(){
		return Status::Running;
	}

}