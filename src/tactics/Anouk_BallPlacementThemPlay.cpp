#include <sstream>
#include <ctime>
#include <chrono>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/tactics/Anouk_BallPlacementThemPlay.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"

#define ROS_LOG_NAME "plays.Anouk_BPus"

namespace rtt {

RTT_REGISTER_TACTIC(Anouk_BallPlacementThemPlay);

Anouk_BallPlacementThemPlay::Anouk_BallPlacementThemPlay(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard){}

void Anouk_BallPlacementThemPlay::Initialize() {

    failed = false;
    succeeded = false;
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

	// Get all available robots
    auto robots = getAvailableRobots();
	// Check if there is a robot available
    if (robots.size() == 0) {
        ROS_WARN_NAMED(ROS_LOG_NAME, "No robots left to claim! Aborting..");
        failed = true;
        return;
    }

	// Get the positions where the ball is, and where it should go
    Vector2 const endPos = LastRef::get().designated_position;
    Vector2 const ballPos = LastWorld::get().ball.pos;

	// Store the current position where the ball should be placed.
	lastBallPlacementPosition = endPos;

	// If the ball is already close enough to the end-point, don't do anything
    if ((endPos - ballPos).length() <= MAX_DISTANCE_FROM_END_POINT) {
        succeeded = true;
        return;
    }



    // ==== Move our robots out of the way if needed ==== //
	// Get the last world
	roboteam_msgs::World world = LastWorld::get();
    // For each robot
    for(auto robot : robots){
		Vector2 pos(world.us.at(robot).pos);
		// Get distance between the robot and the path of the ball
		double distanceToLine = distanceFromPointToLine(ballPos, endPos, pos);

		// If the robot is too close to the path of the ball
		if(distanceToLine < 1){

			// Find projection of pos on line
			Vector2 projection = projectPointOntoLine(ballPos, endPos, pos);

			// Get vector between pos and projection
			Vector2 projectionToPos = projection - pos;

			// Scale vector to length 1. Robots stay 1 meter from the path of the ball
			projectionToPos = projectionToPos.normalize();

			// Get two new positions on both sides of the line
			Vector2 pos1 = projection - projectionToPos;
			Vector2 pos2 = projection + projectionToPos;

			// Find the position closest to the centre of the field. Prevents robots from driving out of the field
			Vector2 newPos = pos1.length() < pos2.length() ? pos1 : pos2;

			// Send role to robot
			{
				/* Create Blackboard */
				bt::Blackboard bb;
				// Set the robot ID
				bb.SetInt("ROBOT_ID", robot);
				bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());
				// Set positioning variables
				ScopedBB(bb, "GoToPos_A")
					.setDouble("angleGoal", 0)
					.setDouble("xGoal"    , newPos.x)
					.setDouble("yGoal"    , newPos.y)
					.setBool("avoidRobots", true)
					.setBool("avoidBall"  , true)
					.setBool("stayAwayFromBall", true)
					.setDouble("maxSpeed" , 1.3);

				/* Create message */
				roboteam_msgs::RoleDirective rd;
				rd.robot_id = robot;
				rd.tree = "rtt_emiel/GoToPos_Nonstop_Role";
				rd.blackboard = bb.toMsg();

				// Send to rolenode
				pub.publish(rd);
			}

		}

    }


}

bool Anouk_BallPlacementThemPlay::ballPlacementSuccessful(){
	/* Rulebook 9.2 */

	bool success = true;

	roboteam_msgs::WorldBall ball = LastWorld::get().ball;

	// No robot within 500mm from the ball
	for(auto robot : LastWorld::get().us){
		Vector2 distanceToBall = Vector2(robot.pos) - Vector2(ball.pos);
		if(distanceToBall.length() < 0.5){
			ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Robot " << robot.id << " too close to ball! " << distanceToBall.length() << "m");
			success = false;
		}
	}

	// Ball is stationary
	if(0.1 < Vector2(ball.vel).length()){
		ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Ball is still moving! " << Vector2(ball.vel).length() << "m/s");
		success = false;
	}

	// Within 100mm from the requested position
	Vector2 distance = Vector2(ball.pos) - LastRef::get().designated_position;
	if(0.1 < distance.length()){
		ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Ball is too far from target position! " << distance.length() << "m");
		success = false;
	}

	return success;

}

bt::Node::Status Anouk_BallPlacementThemPlay::Update() {
    // BallPlacementUsPlay is done when a normal start is signalled.
    // RefStateSwitch will take care of that for us

	// Check if the position where the ball has to be placed hasn't changed while running this play
	// If it has changed, terminate the tree, so that it will start again with the correct position
	Vector2 const endPos = LastRef::get().designated_position;
	if(lastBallPlacementPosition != endPos){
		ROS_INFO_NAMED(ROS_LOG_NAME, "Position of ball placement has changed. Terminating tree..");
		return Status::Failure;
	}

    if (failed) {
        return Status::Failure;
    }

    if (succeeded) {
        return Status::Success;
    }



	if (feedbacks.find(token) != feedbacks.end()) {
        Status status = feedbacks.at(token);
        if (status == Status::Success) {


			if(currentState == PlayStates::MOVING_PLACER){
				ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Placer has moved");
				// Reset token
				token = boost::uuids::nil_uuid();
				// Check if the BallPlacement was a success
				if(ballPlacementSuccessful()){
					ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Ball has been placed successfully!");
					return Status::Success;
				}else{
					ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Ball has been placed unsuccessfully!");
					return Status::Failure;
				}
			}


		} else if (status == Status::Running) {
            // Carry on
        } else {
			ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Ball has been placed unsuccessfully!");
            return Status::Failure;
        }
    }

    return bt::Node::Status::Running;
}


} // rtt

