#include "roboteam_tactics/tactics/Emiel_IndirectUsPlay.h"
#include "roboteam_tactics/Parts.h"

#include <boost/uuid/uuid.hpp>
#include <ros/ros.h>

#define ROS_LOG_NAME "plays.Emiel_IndirectUsPlay"

namespace rtt {

RTT_REGISTER_TACTIC(Emiel_IndirectUsPlay);

Emiel_IndirectUsPlay::Emiel_IndirectUsPlay(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard){}

void Emiel_IndirectUsPlay::Initialize() {
	ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Initializing...");

	// Clear all the feedback tokens
	tokens.clear();

	// Get the default roledirective publisher
	auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
	// Get the last world
	roboteam_msgs::World world = LastWorld::get();
	// Get the available robots
	std::vector<int> robots = getAvailableRobots();
	// Check if enough robots available
	if (getAvailableRobots().size() < 1) {
		ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "No robots available, cannot initialize...");
		return;
	}

	// Get the ID of the keeper
	int keeperID = RobotDealer::get_keeper();

	/// === Initialize keeper === ///
	// =========================
	// Initialize the Keeper
	// =========================
	{
		delete_from_vector(robots, keeperID);

		claim_robot(keeperID);

		roboteam_msgs::RoleDirective rd;
		rd.robot_id = keeperID;
		bt::Blackboard bb;

		bb.SetInt("ROBOT_ID", keeperID);
		bb.SetInt("KEEPER_ID", keeperID);

		// Create message
		rd.tree = "rtt_jim/KeeperRole";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		tokens.push_back(token);
		rd.token = unique_id::toMsg(token);

		// Send to rolenode
		pub.publish(rd);
	}


	/// === Initialize kicker === ///
	// Get the position of the ball
	Vector2 ballPos = Vector2(world.ball.pos);
	// Get the robot closest to the ball
	int ballGetterID = *get_robot_closest_to_point(robots, world, ballPos);
	// =============================
	// Initialize the kicker
	// =============================
	{
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = ballGetterID;
		bt::Blackboard bb;
		claim_robot(ballGetterID);
		delete_from_vector(robots, ballGetterID);

		bb.SetInt("ROBOT_ID", ballGetterID);
		bb.SetInt("KEEPER_ID", keeperID);

		bb.SetBool("GetBall_A_passToBestAttacker", true);
		bb.SetBool("GetBall_A_dontShootAtGoal", true);
		bb.SetBool("GetBall_A_passOn", true);

		// Create message
		rd.tree = "rtt_jim/GetBallRole";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		tokens.push_back(token);
		rd.token = unique_id::toMsg(token);

		// Send to rolenode
		pub.publish(rd);
	}


	/// === Initialize defenders === ///
	int numDefenders = std::min((int)robots.size(), 2);
	// Get position in front of goal
	Vector2 defendPos = LastWorld::get_our_goal_center();// + Vector2(1.5, 0.0);
	// Get angle between goal and ball
	float angle = -(Vector2(LastWorld::get().ball.pos) - Vector2(defendPos)).angle();
	// Get positions to defend
	std::vector<Vector2> positions = RobotPatternGenerator::Line(numDefenders, 0.5, defendPos, angle, 2.5);
	// Assign robots to positions
	std::vector<int> ballDefenders = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, positions, world);

	for (size_t i = 0; i < ballDefenders.size(); i++) {
		int robotID = ballDefenders.at(i);
		delete_from_vector(robots, robotID);
		claim_robot(robotID);

		// Get the position of the ball
		roboteam_msgs::World world = LastWorld::get();
		Vector2 ballPos;
		if (!std::isnan(world.ball.pos.x)) {
			ballPos = world.ball.pos;
		}

		// Calculate angle to ball
		float angleRobotToBall = (ballPos - positions.at(i)).angle();

		/* Create Blackboard */
		bt::Blackboard bb;
		// Set the robot ID
		bb.SetInt("ROBOT_ID", robotID);
		bb.SetInt("KEEPER_ID", keeperID);

		// Set positioning variables
		ScopedBB(bb, "GoToPos_A")
			.setDouble("angleGoal", angleRobotToBall)
			.setDouble("xGoal"    , positions.at(i).x)
			.setDouble("yGoal"    , positions.at(i).y)
			.setBool("avoidRobots", true)
			.setBool("avoidBall"  , true)
			.setBool("stayAwayFromBall"  , true)
			.setBool("avoidDefenseAreas"  , true)
		;

		/* Create message */
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = robotID;
		rd.tree = "rtt_emiel/GoToPos_Nonstop_Role";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		rd.token = unique_id::toMsg(token);

		// Send to rolenode
		pub.publish(rd);
	}


	/// === Initialize attackers === ///
	int numAttackers = (int)robots.size();
	// Get position in front of goal
	Vector2 attackPos = LastWorld::get_their_goal_center();

	// Get positions to attack
	positions = RobotPatternGenerator::Circle(numAttackers, M_PI/2, 4, attackPos, M_PI, 0, 1);
	// Assign robots to positions
	std::vector<int> attackers = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, positions, world);

	for (int i = 0; i < numAttackers; i++) {

		int robotID = attackers.at(i);
		delete_from_vector(robots, robotID);
		claim_robot(robotID);

		roboteam_msgs::RoleDirective rd;
		rd.robot_id = robotID;
		bt::Blackboard bb;

		// Set the robot ID
		bb.SetInt("ROBOT_ID", robotID);
		bb.SetInt("KEEPER_ID", keeperID);

		// bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", false);
		bb.SetBool("ReceiveBall_A_computePoint", true);
		bb.SetDouble("ReceiveBall_A_computePointCloseToX", positions.at(i).x);
		bb.SetDouble("ReceiveBall_A_computePointCloseToY", positions.at(i).y);
		bb.SetBool("ReceiveBall_A_setSignal", true);
		bb.SetBool("ReceiveBall_A_shouldFail", true);
		bb.SetDouble("ReceiveBall_A_acceptableDeviation", 0.4);

		// Create message
		rd.tree = "rtt_jim/DirectStrikerRole";
		rd.blackboard = bb.toMsg();

		// Add random token and save it for later
		boost::uuids::uuid token = unique_id::fromRandom();
		tokens.push_back(token);
		rd.token = unique_id::toMsg(token);

		// Send to rolenode
		pub.publish(rd);
	}


}

bt::Node::Status Emiel_IndirectUsPlay::Update() {

	// Wait for the ball to move...

	return Status::Running;

}

} //namespace rtt