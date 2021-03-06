#include "roboteam_tactics/tactics/Anouk_PrepareDirectThem.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"
#include "roboteam_tactics/utils/RobotsToDefendFinder.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Anouk_PrepareDirectThem"

namespace rtt {

    RTT_REGISTER_TACTIC(Anouk_PrepareDirectThem);

    Anouk_PrepareDirectThem::Anouk_PrepareDirectThem(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}

	void Anouk_PrepareDirectThem::init(){

		/// === Check if reinitialization is needed === ///
		// Get the new robots that should be intercepted
		std::vector<int> robotsToInterceptNew = RobotsToDefendFinder::GetRobotsToDefend(0.5, true);
		// Get the new robots that should be defended
		std::vector<int> robotsToDefendNew = RobotsToDefendFinder::GetRobotsToDefend(3.0, true);

		bool shouldReinitialize = false;
		// Check if the size of the vectors have changed
		if (robotsToDefend.size() == robotsToDefendNew.size() && robotsToIntercept.size() == robotsToInterceptNew.size()){
			// Check if the vector robotsToDefend has changed
			for (size_t i = 0; i < robotsToDefendNew.size(); i++) {
				if (robotsToDefend[i] != robotsToDefendNew[i]){
					ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Not Equal: Status Success ");
					shouldReinitialize = true;
				}
			}
			// Check if the vector robotsToIntercept has changed
			for (size_t i = 0; i < robotsToInterceptNew.size(); i++) {
				if (robotsToIntercept[i] != robotsToInterceptNew[i]) {
					ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Not Equal: Status Success ");
					shouldReinitialize = true;
				}
			}
		} else {
			ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Not Equal size: [" << robotsToDefend.size() << " -> " << robotsToDefendNew.size() << ", " << robotsToIntercept.size() << " -> " << robotsToInterceptNew.size() << "]");
			shouldReinitialize = true;
		}

		if(!shouldReinitialize)
			return;

		/// === Start the reinitialization === //
		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Reinitializing...");
		// Store the robots to defend and intercept
		robotsToDefend = robotsToDefendNew;
		robotsToIntercept = robotsToInterceptNew;

		// Print the robots that should be defended and intercepted
		std::stringstream vectorStr;
		std::copy(robotsToDefend.begin(), robotsToDefend.end(), std::ostream_iterator<int>(vectorStr, ", "));
		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Robots to defend    : [" << vectorStr.str().c_str() << "]");
		vectorStr.str("");  // https://stackoverflow.com/questions/20731/how-do-you-clear-a-stringstream-variable
		vectorStr.clear();	// https://stackoverflow.com/questions/2848087/how-to-clear-stringstream
		std::copy(robotsToIntercept.begin(), robotsToIntercept.end(), std::ostream_iterator<int>(vectorStr, ", "));
		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Robots to intercept : [" << vectorStr.str().c_str() << "]");

		// Release all previously claimed robots
		release_robots(get_claimed_robots());
		// Get all the available robots
		std::vector<int> robots = getAvailableRobots();
		int robotsLeft = (int)robots.size();

		// Number of ball defenders
		int numBallDefenders = std::min(robotsLeft, 2);
		robotsLeft -= numBallDefenders;

		// Number of robot defenders
		int numRobotDefenders = std::min(robotsLeft, (int)robotsToDefend.size());
		robotsLeft -= numRobotDefenders;

		// Number of ball interceptors
		int numBallInterceptors = std::min(robotsLeft, (int)robotsToIntercept.size());
		robotsLeft -= numBallInterceptors;

		// Add the remaining robots to ball defenders
		numBallDefenders += robotsLeft;
		robotsLeft = 0;

		// Get the last world
		roboteam_msgs::World world = LastWorld::get();
		// Get the position of the ball
		Vector2 ballPos(world.ball.pos);
		// Get the position of our goal
		Vector2 goalPos(LastWorld::get_our_goal_center());

		double angleGoalToBall = -(ballPos - goalPos).angle();

		// Calculate the positions of the ball defenders
		std::vector<Vector2> defenderCoords = RobotPatternGenerator::Line(numBallDefenders, 0.20 * (numBallDefenders - 1), goalPos, angleGoalToBall, 1.8);

		// Place the ball defenders and robot defender, also claims the robots
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		Emiel_Prepare::prepare(*refState, defenderCoords, robotsToDefend, robotsToIntercept);

		// Initialize the robots that are left


	}

    void Anouk_PrepareDirectThem::Initialize() {
		ROS_INFO_NAMED(ROS_LOG_NAME, "Initializing Anouk_PrepareDirectThem");

        // Check if this tree is used for the correct RefState, DIRECT_FREE_THEM or INDIRECT_FREE_THEM
        boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
        if (refState != RefState::DIRECT_FREE_THEM && refState != RefState::INDIRECT_FREE_THEM) {
            ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for DIRECT_FREE_THEM or INDIRECT_FREE_THEM");
        }
		// Clear these vectors, to make sure that the role directives are actually initialized
		robotsToDefend.clear();
		robotsToIntercept.clear();

		Anouk_PrepareDirectThem::init();

    }

	///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// WATCH OUT! Update can return SUCCESS or FALSE, ONLY when the ball has moved!
	// Returning SUCCESS or FALSE triggers RefState::NORMAL_START, starting the match
	///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    bt::Node::Status Anouk_PrepareDirectThem::Update() {

		Anouk_PrepareDirectThem::init();

		if(Vector2(LastWorld::get().ball.vel).length() > 0.5)
			return Status::Success;

		return Status::Running;

    }

}
