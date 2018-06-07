#include "roboteam_tactics/tactics/Anouk_StopPlay.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Anouk_StopPlay"

namespace rtt {

    RTT_REGISTER_TACTIC(Anouk_StopPlay);

    Anouk_StopPlay::Anouk_StopPlay(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}



	void Anouk_StopPlay::init(){
		/// === Start the reinitialization === //
		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Reinitializing...");


		// Release all previously claimed robots
		release_robots(get_claimed_robots());
		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		int robotsLeft = (int)robots.size();

		// Number of ball defenders
		int numGoalDefenders = std::min(robotsLeft, 5);
		robotsLeft -= numGoalDefenders;

		// Number of robot defenders
		int numBallDefenders = robotsLeft;



		// Get the last world
		roboteam_msgs::World world = LastWorld::get();
		// Get the position of the ball
		Vector2 ballPos(world.ball.pos);
		// Get the position of our goal
		Vector2 goalPos(LastWorld::get_our_goal_center());

		double angleGoalToBall = -(ballPos - goalPos).angle();

		// Calculate the positions of the ball defenders
		std::vector<Vector2> goalDefenderCoords = RobotPatternGenerator::Line(numGoalDefenders, 2, goalPos, 0, 1.2);
        std::vector<Vector2> ballDefenderCoords = RobotPatternGenerator::Line(numBallDefenders, 4, goalPos, 0, 3.8);

        // Put all positions into one array
        std::vector<Vector2> positions;
        positions.insert(std::end(positions), std::begin(goalDefenderCoords), std::end(goalDefenderCoords));
        positions.insert(std::end(positions), std::begin(ballDefenderCoords), std::end(ballDefenderCoords));


		// Place the ball defenders and robot defender, also claims the robots
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		Emiel_Prepare::prepare(*refState, positions);


	}

    void Anouk_StopPlay::Initialize() {

        // Check if this tree is used for the correct RefState, DIRECT_FREE_THEM or INDIRECT_FREE_THEM
        boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
        if (refState != RefState::STOP) {
            ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for STOP");
        }

		Anouk_StopPlay::init();

    }

    bt::Node::Status Anouk_StopPlay::Update() {

		return Status::Running;

    }
}
