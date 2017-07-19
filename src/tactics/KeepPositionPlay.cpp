#include "roboteam_tactics/tactics/KeepPositionPlay.h"
#include "roboteam_tactics/utils/RobotDealer.h"

namespace rtt {

RTT_REGISTER_TACTIC(KeepPositionPlay);

KeepPositionPlay::KeepPositionPlay(std::string name, bt::Blackboard::Ptr bb) : Tactic(name, bb) {}

void KeepPositionPlay::Initialize() {

	auto available = getAvailableRobots();

	auto& pub = GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

	for (int id : available) {
		if (id == GetInt("KEEPER_ID")) continue;
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = id;
		rd.tree = "rtt_dennis/KeepPositionRole";
		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", id);
		rd.blackboard = bb.toMsg();
		pub.publish(rd);
	}

	claim_robots(available);
}

bt::Node::Status KeepPositionPlay::Update() {
	return Status::Running;
}

}
