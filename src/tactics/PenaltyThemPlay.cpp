#include "roboteam_tactics/tactics/PenaltyThemPlay.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

RTT_REGISTER_TACTIC(PenaltyThemPlay);

PenaltyThemPlay::PenaltyThemPlay(std::string name, bt::Blackboard::Ptr bb) : Tactic(name, bb),
		shooterId{-1}, shootoutPenalty{false}, valid{false} {}

void PenaltyThemPlay::Initialize() {
	if (HasBool("isShootout")) {
		shootoutPenalty = GetBool("isShootout");
	} else {
		ROS_ERROR("PenaltyThemPlay: No bool 'isShootout' in blackboard!");
		return;
	}

	int keeperId = RobotDealer::get_keeper();

	if (!RobotDealer::get_keeper_available()) {
		ROS_WARN("PenaltyThemPlay: Keeper is unavailable, but since this is"
				" a penalty I'm going to send him commands anyway");
	} else {
		claim_robot(keeperId);
	}

	valid = true;

	auto& pub = GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

	{
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = keeperId;
		rd.tree = "rtt_dennis/DefendPenaltyRole";

		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", keeperId);
		bb.SetInt("KEEPER_ID", keeperId);
		rd.blackboard = bb.toMsg();
		pub.publish(rd);
	}
}

bt::Node::Status PenaltyThemPlay::Update() {
	if (!valid) return Status::Invalid;
	return Status::Running;
}
}
