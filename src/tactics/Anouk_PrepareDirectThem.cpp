#include "roboteam_tactics/tactics/Anouk_PrepareDirectThem.h"

#include <tuple>
#include <vector>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/RobotPatternGenerator.h"

#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Anouk_PrepareDirectThem"

namespace rtt {

    RTT_REGISTER_TACTIC(Anouk_PrepareDirectThem);

    Anouk_PrepareDirectThem::Anouk_PrepareDirectThem(std::string name, bt::Blackboard::Ptr blackboard) : Emiel_Prepare(name, blackboard) {}


    std::vector<int> Anouk_PrepareDirectThem::GetRobotsToDefend(double minDangerScore){

        // Get the last world
        roboteam_msgs::World world = LastWorld::get();

        // Get the position of the ball
        Vector2 ballPos(world.ball.pos);


        // === Get their robot that is the closest to the ball === //
        int oppKicker = -1; // Holds the kicker ID
        double oppClosestDistance = 9999; // Holds the closest distance

        for(size_t i = 0; i < world.them.size(); i++){
            // Get the distance between the ball and the current opponent
            double distanceToBall = (Vector2(world.them.at(i).pos) - ballPos).length();
            // If this distance is closer than previous distances, store it
            if(distanceToBall < oppClosestDistance ){
                oppClosestDistance = distanceToBall;
                oppKicker = i;
            }
        }
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1, ROS_LOG_NAME, "Robot closest to ball : " << oppKicker);


        // === Find the most dangerous opponents, excluding the kicker ===
        // Vector to hold the new robots to defend
        std::vector<int> newRobotsToDefend;
        // For each opponent in the world
        for (size_t i = 0; i < world.dangerList.size(); i++) {
            // Get the opponent
            roboteam_msgs::WorldRobot opp = world.dangerList.at(i);
            // If the robot is the kicker, ignore
            if(opp.id == oppKicker)
                continue;
            // Get danger score of the robot
            float dangerScore = world.dangerScores.at(i);

            // If the danger score of the robot is too low, ignore it
            if(dangerScore < minDangerScore)
                continue;

            // The robot should be defended
            newRobotsToDefend.push_back(opp.id);
        }

        return newRobotsToDefend;
    }

	void Anouk_PrepareDirectThem::init(){

		/// === Check if reinitialization is needed === ///
		// Get the new robots that should be intercepted
		std::vector<int> robotsToInterceptNew = GetRobotsToDefend(0.0);
		// Get the new robots that should be defended
		std::vector<int> robotsToDefendNew = GetRobotsToDefend(3.0);

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

		// Print the robots that should be defended
		std::stringstream vectorStr;
		std::copy(robotsToDefend.begin(), robotsToDefend.end(), std::ostream_iterator<int>(vectorStr, " "));
		ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Robots to defend : " << vectorStr.str().c_str());

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
		int numBallInterceptors = robotsLeft;


		// Get the last world
		roboteam_msgs::World world = LastWorld::get();
		// Get the position of the ball
		Vector2 ballPos(world.ball.pos);
		// Get the position of our goal
		Vector2 goalPos(LastWorld::get_our_goal_center());

		double angleGoalToBall = -(ballPos - goalPos).angle();

		// Calculate the positions of the ball defenders
		std::vector<Vector2> defenderCoords = RobotPatternGenerator::Line(numBallDefenders, 0.20, goalPos, angleGoalToBall, 1.8);

		// Place the ball defenders and robot defender, also claims the robots
		boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
		Emiel_Prepare::prepare(*refState, defenderCoords, robotsToDefend, robotsToIntercept);

		// Initialize the robots that are left


	}

    void Anouk_PrepareDirectThem::Initialize() {

        // Check if this tree is used for the correct RefState, DIRECT_FREE_THEM or INDIRECT_FREE_THEM
        boost::optional<rtt::RefState> refState = LastRef::getCurrentRefCommand();
        if (refState != RefState::DIRECT_FREE_THEM && refState != RefState::INDIRECT_FREE_THEM) {
            ROS_WARN_NAMED(ROS_LOG_NAME, "Watch out! This strategy is specifically designed for DIRECT_FREE_THEM or INDIRECT_FREE_THEM");
        }

		Anouk_PrepareDirectThem::init();

    }

	///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// WATCH OUT! Update can return SUCCESS or FALSE, ONLY when the ball has moved!
	// Returning SUCCESS or FALSE triggers RefState::NORMAL_PLAY, starting the match
	///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    bt::Node::Status Anouk_PrepareDirectThem::Update() {

		Anouk_PrepareDirectThem::init();

		if(Vector2(LastWorld::get().ball.vel).length() > 0.5)
			return Status::Success;

		return Status::Running;

    }
}
