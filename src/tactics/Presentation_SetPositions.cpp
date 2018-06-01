#include "roboteam_tactics/tactics/Presentation_SetPositions.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Presentation_SetPositions"

namespace rtt {

	RTT_REGISTER_TACTIC(Presentation_SetPositions);

	Presentation_SetPositions::Presentation_SetPositions(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}

	void Presentation_SetPositions::Initialize(){

		// Check if this tree is used for the correct RefState, PREPARE_PENALTY_US
		boost::optional<rtt::RefState> refState = RefState::HALT;
		// if(refState != RefState::PREPARE_PENALTY_US){
		// 	ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for PREPARE_PENALTY_US");
		// }

		std::string our_color;
		ros::param::get("our_color", our_color);
		bool blue_team = (our_color == "blue");

		Vector2 goalCenter;
		if (blue_team) {
			goalCenter = LastWorld::get_our_goal_center();
		} else {
			goalCenter = LastWorld::get_their_goal_center();
		}

		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();

		// Holds positions
		std::vector<Vector2> positions;
		int formation = 0;
		if (HasInt("formation")) {
			formation = GetInt("formation");
		}

		// positions in order of importance
		if (formation == 0) { // standing at goal line
			if (blue_team) { // NOTE: coordinates flip sign for blue team
				positions.push_back(goalCenter + Vector2(0.1, 1.0));
				positions.push_back(goalCenter + Vector2(0.1, 1.5));
				positions.push_back(goalCenter + Vector2(0.1, 2.0));
			} else {
				positions.push_back(goalCenter + Vector2(-0.1, 1.0));
				positions.push_back(goalCenter + Vector2(-0.1, 1.5));
				positions.push_back(goalCenter + Vector2(-0.1, 2.0));
			}
		} else if (formation == 1) { // behind triangle
			if (blue_team) {
				positions.push_back(goalCenter + Vector2(0.4, 0.0));
				positions.push_back(goalCenter + Vector2(0.25, 0.25));
				positions.push_back(goalCenter + Vector2(0.4, 0.25));
			} else {
				positions.push_back(goalCenter + Vector2(-0.25, 0.0));
				positions.push_back(goalCenter + Vector2(-0.25, 0.25));
				positions.push_back(goalCenter + Vector2(-0.4, 0.25));
			}
		} else if (formation == 2) { // escape triangle on both sides step 1
			if (blue_team) {
				positions.push_back(goalCenter + Vector2(0.4, 0.0));
				positions.push_back(goalCenter + Vector2(0.25, 2.0));
				positions.push_back(goalCenter + Vector2(0.4, 0.25));
			} else {
				positions.push_back(goalCenter + Vector2(-0.25, 0.0));
				positions.push_back(goalCenter + Vector2(-0.25, 2.0));
				positions.push_back(goalCenter + Vector2(-0.4, 0.25));
			}
		} else if (formation == 3) { // escape triangle on both sides step 2
			if (blue_team) {
				positions.push_back(goalCenter + Vector2(0.4, 0.0));
				positions.push_back(goalCenter + Vector2(0.25, 2.0));
				positions.push_back(goalCenter + Vector2(0.25, 1.5));
			} else {
				positions.push_back(goalCenter + Vector2(-0.25, 0.0));
				positions.push_back(goalCenter + Vector2(-0.25, 2.0));
				positions.push_back(goalCenter + Vector2(-0.25, 1.5));
			}
		} else if (formation == 4) { // escape triangle on both sides step 3
			if (blue_team) {
				positions.push_back(goalCenter + Vector2(0.25, 1.0));
				positions.push_back(goalCenter + Vector2(0.25, 2.0));
				positions.push_back(goalCenter + Vector2(0.25, 1.5));
			} else {
				positions.push_back(goalCenter + Vector2(-0.25, 1.0));
				positions.push_back(goalCenter + Vector2(-0.25, 2.0));
				positions.push_back(goalCenter + Vector2(-0.25, 1.5));
			}
		} else if (formation == 5) { // drive forward
			if (blue_team) {
				positions.push_back(goalCenter + Vector2(1.3, 1.0));
				positions.push_back(goalCenter + Vector2(1.3, 2.0));
				positions.push_back(goalCenter + Vector2(1.3, 1.5));
			} else {
				positions.push_back(goalCenter + Vector2(-1.3, 1.0));
				positions.push_back(goalCenter + Vector2(-1.3, 2.0));
				positions.push_back(goalCenter + Vector2(-1.3, 1.5));
			}
		} else if (formation == 6) { // blue keeper, yellow attacker, rest stationary
			if (blue_team) {
				positions.push_back(goalCenter + Vector2(0.1, 1.0));
				positions.push_back(goalCenter + Vector2(0.1, 1.5));
				positions.push_back(goalCenter + Vector2(0.1, 0.0));
			} else {
				positions.push_back(goalCenter + Vector2(-0.1, 1.0));
				positions.push_back(goalCenter + Vector2(-0.1, 1.5));
				positions.push_back(goalCenter + Vector2(-1.5, 0.0));
			}
		} else if (formation == 7) { // blue keeper, 2 blue defenders in the way of shot, yellow behind ball
			if (blue_team) {
				positions.push_back(goalCenter + Vector2(0.1, 0.0));
				positions.push_back(Vector2(-0.2, 1.15));
				positions.push_back(Vector2(-0.05, 1.1));
			} else {
				positions.push_back(goalCenter + Vector2(-0.1, 1.0));
				positions.push_back(goalCenter + Vector2(-0.1, 1.5));
				positions.push_back(Vector2(-0.2, -2.1));
			}
		} else if (formation == 8) { // blue keeper, 2 blue defenders in the way of shot and pass, yellow behind ball
			if (blue_team) {
				positions.push_back(goalCenter + Vector2(0.1, 0.0));
				positions.push_back(Vector2(-0.2, 1.15));
				positions.push_back(Vector2(0.2, 1.1));
			} else {
				positions.push_back(goalCenter + Vector2(-0.1, 1.0));
				positions.push_back(goalCenter + Vector2(-0.1, 1.5));
				positions.push_back(Vector2(-0.2, -2.1));
			}
		}

		// 3 or less robots
		int numRobots = robots.size();
		int numPositions = 3;
		if (numRobots < numPositions) {
			for (int i = 0; i < numPositions - numRobots; i++) {
				positions.pop_back();
			}
		}


		// // Calculate positions for defenders and attackers
		// /* Creates a line of 2? robots, 1 meter wide, 2 meters in front of our goal center */
		// std::vector<Vector2> defenderCoords = RobotPatternGenerator::Line(numDefenders, 1, LastWorld::get_our_goal_center(), 0, 2);
		// /* Creates a circle of 5? robots, covering M_PI/2 rad, in a circle with radius 3, around their goal center, with a M_PI rad phase shift */
		// std::vector<Vector2> attackerCoords = RobotPatternGenerator::Circle(numAttackers, M_PI/2, 3, LastWorld::get_their_goal_center(), M_PI, 0, 0);

		// // Put all positions into one array
		// positions.insert(std::end(positions), std::begin(defenderCoords), std::end(defenderCoords));
		// positions.insert(std::end(positions), std::begin(attackerCoords), std::end(attackerCoords));

		Emiel_Prepare::prepare(*refState, positions);


	}

	bt::Node::Status Presentation_SetPositions::Update(){
		return Status::Running;
	}

}