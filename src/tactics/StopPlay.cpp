#include "roboteam_tactics/tactics/StopPlay.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/utils/RobotDealer.h"

#define RTT_CURRENT_DEBUG_TAG(StopPlay)

namespace rtt {

RTT_REGISTER_TACTIC (StopPlay);

StopPlay::StopPlay(std::string name, bt::Blackboard::Ptr bb) :
		Tactic(name, bb), canRun(false) {

}

void sendRoleDirectives(const std::vector<int>& ids) {
	auto& pub = GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
	for (int id : ids) {
		auto bot = getWorldBot(id);
		if (!bot) {
			return;
		}
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = id;
		rd.tree = "rtt_dennis/KeepPositionRole";

		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", id);
		bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());
		bb.SetDouble("GoToPos_A_xGoal", bot->pos.x);
		bb.SetDouble("GoToPos_A_yGoal", bot->pos.y);
		bb.SetDouble("GoToPos_A_angleGoal", bot->angle);
		bb.SetBool("GoToPos_A_avoidRobots", true);

		rd.blackboard = bb.toMsg();
		pub.publish(rd);
	}
}

bt::Node::Status StopPlay::Update() {
	std::vector<int> claimed = get_claimed_robots();
	std::vector<int> toRemove, toAdd;
	const auto world = LastWorld::get();
	const auto us = world.us;

	for (int id : claimed) {
		if (!getWorldBot(id)) {
			ROS_WARN("StopPlay: Robot %d was not found anymore", id);
			toRemove.push_back(id);
		}
	}

	for (const auto& bot : us) {
		if (bot.id != (unsigned) RobotDealer::get_keeper() &&
				std::find(claimed.begin(), claimed.end(), bot.id) == claimed.end()) {
			ROS_WARN("StopPlay: New robot %d found", bot.id);
			toAdd.push_back(bot.id);
		}
	}

	if (!release_robots(toRemove)) {
		ROS_ERROR("StopPlay: Unable to release all vanished robots!");
	} else {
		auto& pub = GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
		for (int id : toRemove) {
			roboteam_msgs::RoleDirective rd;
			rd.robot_id = id;
			rd.tree = roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE;

			bt::Blackboard bb;
			bb.SetInt("ROBOT_ID", id);
			bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());

			rd.blackboard = bb.toMsg();
			pub.publish(rd);
		}
	}

	if (!claim_robots(toAdd)) {
		ROS_ERROR("StopPlay: Unable to claim all new robots!");
	} else {
		sendRoleDirectives(toAdd);
	}

	sendRoleDirectives(toAdd);

	return Status::Running;
}

void StopPlay::Initialize() {
	std::vector<int> available = getAvailableRobots();
	if (available.size() == 0) {
		return;
	}

	claim_robots(available);
	sendRoleDirectives(available);
}

}
