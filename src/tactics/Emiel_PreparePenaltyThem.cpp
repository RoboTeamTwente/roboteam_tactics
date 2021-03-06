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

		// Get the latest world
		const roboteam_msgs::World& world = LastWorld::get();

		// Check if this tree is used for the correct RefState, PREPARE_PENALTY_THEM
		// TODO Get the current refState not from LastRef, but from the RefStateSwitch. RefStateSwitch can correctly return DEFEND_PENALTY, LastRef cannot! Function = getExtentedState();
		refState = LastRef::getCurrentRefCommand();
		if(refState != RefState::PREPARE_PENALTY_THEM && refState != RefState::NORMAL_START){
			ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_PENALTY_THEM or DEFEND_PENALTY. Current : " << rtt::refStateToString(*refState));
		}

		// Check if ball is on their side, which would mean shootout
		if(world.ball.pos.x > 0){
			ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "PENALTY SHOOTOUT DETECTED! Initializing defense");
			Emiel_PreparePenaltyThem::initShootout();
			return;
		}

		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		// Number of defenders that should stay behind
		int numDefenders = std::min((int)robots.size(), 5);
		// Number of attackers that should go to the front
		int numAttackers = std::max((int)robots.size()-numDefenders, 0);

		// === Calculate positions for defenders and attackers === //
        // Robots should be placed behind a line parallel to the goal line and 400 mm behind the penalty mark. Penalty mark : B=1m, A=1.2m
        Vector2 linePosition = (Vector2)LastWorld::get_our_goal_center() + Vector2(1.2 + 0.4 + 0.2, 0); // Penalty mark A + 400mm + 200mm padding
        float lineWidth = LastWorld::get_field().field_width * 0.8;

		std::vector<Vector2> defenderCoords = RobotPatternGenerator::Line(numDefenders, lineWidth, linePosition, 0, 0);
		std::vector<Vector2> attackerCoords = RobotPatternGenerator::Line(numAttackers, lineWidth, Vector2(0, 0), 0, 0);

		// Put all positions into one array
		std::vector<Vector2> positions;
		positions.insert(std::end(positions), std::begin(defenderCoords), std::end(defenderCoords));
		positions.insert(std::end(positions), std::begin(attackerCoords), std::end(attackerCoords));

		if(refState == RefState::PREPARE_PENALTY_THEM)
			Emiel_Prepare::prepare(*refState, positions);
		if(refState == RefState::NORMAL_START)
			Emiel_Prepare::prepare(RefState::DEFEND_PENALTY, positions);

	}

	bt::Node::Status Emiel_PreparePenaltyThem::Update(){

		// TODO Get the current refState not from LastRef, but from the RefStateSwitch. RefStateSwitch can correctly return DEFEND_PENALTY, LastRef cannot!
		if(refState == RefState::PREPARE_PENALTY_THEM) {
			return Status::Running;
		}
		// RefState::DEFEND_PENALTY, see TODO
		if(refState == RefState::NORMAL_START) {
			// Wait for the ball to move. Rulebook 14.3 : "The ball is in play when it is kicked and moves forward"
			if(LastWorld::get().ball.vel.x < -0.5){
				ROS_DEBUG_NAMED(ROS_LOG_NAME, "Ball is moving! Returning Status::Success");
				return Status::Success;
			}else{
				ROS_DEBUG_THROTTLE_NAMED(1, ROS_LOG_NAME, "Ball is stationary. Returning Status::Running");
				return Status::Running;
			}
		}

		ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Should not be here! " << rtt::refStateToString(*refState));
		return Status::Failure;
	}






void Emiel_PreparePenaltyThem::initShootout(){

	// Get all the available robots
	std::vector<int> robots = getAvailableRobots();
	int numRobots = robots.size();

	// Calculate positions for defenders and attackers
	std::vector<Vector2> positions = RobotPatternGenerator::Line(numRobots , 1, Vector2(5.5, 4), 0, 0);

	// Initialize positions
	Emiel_Prepare::prepare(RefState::DEFEND_PENALTY, positions);

}


}