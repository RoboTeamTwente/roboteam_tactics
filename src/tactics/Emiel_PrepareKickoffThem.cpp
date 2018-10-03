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
		int robotsLeft = (int)robots.size();

		// Number of ball defenders for the goal
		int numBallDefendersGoal = std::min(robotsLeft, 2);
		robotsLeft -= numBallDefendersGoal;

		// Number of ball defenders
		int numBallDefenders = std::min(robotsLeft, 3);
		robotsLeft -= numBallDefenders;

		// Number of ball defenders
		int numSideDefenders = robotsLeft;

		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "numBallDefendersGoal=" << numBallDefendersGoal << ", numBallDefenders="<<numBallDefenders << ", numSideDefenders="<<numSideDefenders);

		// Get the last world
		roboteam_msgs::World world = LastWorld::get();
		// Get field dimensions
        auto field = LastWorld::get_field();
		// Get the position of the ball
		Vector2 ballPos(world.ball.pos);
		// Get the position of our goal
		Vector2 goalPos(LastWorld::get_our_goal_center());


		double angleGoalToBall = -(ballPos - goalPos).angle();
		std::cout<<angleGoalToBall<<std::endl;

		// Calculate the positions of the ball defenders for the goal
		std::vector<Vector2> ballDefenderGoalCoords = RobotPatternGenerator::Line(numBallDefendersGoal, 0.5, goalPos, angleGoalToBall, 0.4 * field.field_length);

		// Calculate the positions of the ball defenders
		std::vector<Vector2> ballDefenderCoords = RobotPatternGenerator::Line(numBallDefenders, 1.0, goalPos, angleGoalToBall, std::min(0.4 * field.field_length, field.field_width / 2 * 0.9));

		// Calculate the positions of the side robots
        float lineWidth = LastWorld::get_field().field_width * 0.9;
		std::vector<Vector2> sideDefenderCoords = RobotPatternGenerator::Line(numSideDefenders, lineWidth, Vector2(0, 0), 0, -1.0);

		// Put all positions into one array
		std::vector<Vector2> positions;
		positions.insert(std::end(positions), std::begin(ballDefenderGoalCoords), std::end(ballDefenderGoalCoords));
		positions.insert(std::end(positions), std::begin(ballDefenderCoords), std::end(ballDefenderCoords));
		positions.insert(std::end(positions), std::begin(sideDefenderCoords), std::end(sideDefenderCoords));

		Emiel_Prepare::prepare(*refState, positions);

	}

	bt::Node::Status Emiel_PrepareKickoffThem::Update(){
		return Status::Running;
	}

}