#include "roboteam_tactics/tactics/PenaltyUsPlay.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

RTT_REGISTER_TACTIC(PenaltyUsPlay);

PenaltyUsPlay::PenaltyUsPlay(std::string name, bt::Blackboard::Ptr bb) : Tactic(name, bb),
		shooterId{-1}, shootoutPenalty{false}, valid{false} {}

void PenaltyUsPlay::Initialize() {
	if (HasBool("isShootout")) {
		shootoutPenalty = GetBool("isShootout");
	} else {
		ROS_ERROR("PenaltyUsPlay: No bool 'isShootout' in blackboard!");
		return;
	}

	auto available = RobotDealer::get_available_robots();
	if (available.size() == 0) {
		ROS_ERROR("PenaltyUsPlay: No shooter available!");
		return;
	}
	shooterId = available.at(0);
	RobotDealer::claim_robots(available);

	valid = true;

	auto& pub = GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

	{
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = shooterId;
		rd.tree = "rtt_dennis/PenaltyShooter";

		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", shooterId);
		bb.SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		bb.SetBool("isShootout", shootoutPenalty);
		bb.SetDouble("Dribble_A_goalx", shootoutPenalty ? -1.5 : 3.5);
		bb.SetDouble("goalx", shootoutPenalty ? -1.5 : 3.5);
		rd.blackboard = bb.toMsg();
		rd.token = unique_id::toMsg(unique_id::fromRandom());
		pub.publish(rd);
	}

	double maxX = shootoutPenalty ? -2.0 : 3.0;
	int moveCount = 0;

	for (int id : available) {
		if (id == shooterId || id == GetInt("KEEPER_ID")) continue;

		auto bot = *getWorldBot(id);
		bool shouldMove = bot.pos.x > maxX;

		roboteam_msgs::RoleDirective rd;
		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", id);
		bb.SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		if (shouldMove) {
			rd.tree = "rtt_dennis/GoToPosAndStay";
			bb.SetDouble("GoToPos_A_xGoal", maxX);
			bb.SetDouble("GoToPos_A_yGoal", -2 + moveCount++);
			bb.SetDouble("GoToPos_A_angleGoal", -M_PI);
		} else {
			rd.tree = roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE;
		}
		rd.blackboard = bb.toMsg();
		rd.token = unique_id::toMsg(unique_id::fromRandom());
		pub.publish(rd);
	}
}

bt::Node::Status PenaltyUsPlay::Update() {
	if (!valid) return Status::Invalid;
/*
	double ballX = LastWorld::get().ball.pos.x;
	bool shotTaken = ballX > (shootoutPenalty ? 2 : -4);
	return shotTaken ? Status::Success : Status::Running;
*/
	return Status::Running;
}

}
