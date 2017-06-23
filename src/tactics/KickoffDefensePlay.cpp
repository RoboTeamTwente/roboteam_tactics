#include "roboteam_tactics/tactics/KickoffDefensePlay.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_msgs/WorldRobot.h"
#include <vector>

#define RTT_CURRENT_DEBUG_TAG KickoffDefensePlay

namespace rtt {

RTT_REGISTER_TACTIC(KickoffDefensePlay);

KickoffDefensePlay::KickoffDefensePlay(std::string name, bt::Blackboard::Ptr blackboard)
	: Tactic(name, blackboard) {}


struct ProximitySorter {
	bool operator()(const roboteam_msgs::WorldRobot& a, const roboteam_msgs::WorldRobot& b) {
		return a.pos.x < b.pos.x;
	}
};

std::vector<roboteam_msgs::WorldRobot> getRobotsByProximity() {
	auto them = LastWorld::get().them;
	std::sort(them.begin(), them.end(), ProximitySorter{});
	return them;
}

void KickoffDefensePlay::Initialize() {
	RTT_DEBUGLN("KickoffDefensePlay initializing");
	const auto availableBots = RobotDealer::get_available_robots();
	if (availableBots.size() < 3) {
		RTT_DEBUGLN("Not enough robots avaialble...");
		return;
	}

	const auto sortedOpponents = getRobotsByProximity();

	int idA = availableBots.at(0);
	int idB = availableBots.at(1);
	int idC = availableBots.at(2);

	RobotDealer::claim_robots({idA, idB, idC});

	auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

	{
		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", idA);
		bb.SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		bb.SetInt("targetId", sortedOpponents.at(0).id);
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = idA;
		rd.tree = "rtt_dennis/MirrorOpponentRole";
		rd.blackboard = bb.toMsg();

		boost::uuids::uuid token = unique_id::fromRandom();
		tokens.push_back(token);
		rd.token = unique_id::toMsg(token);
		pub.publish(rd);
	}

	{
		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", idB);
		bb.SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		bb.SetInt("targetId", sortedOpponents.at(1).id);
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = idB;
		rd.tree = "rtt_dennis/MirrorOpponentRole";
		rd.blackboard = bb.toMsg();

		boost::uuids::uuid token = unique_id::fromRandom();
		tokens.push_back(token);
		rd.token = unique_id::toMsg(token);
		pub.publish(rd);
	}

	{
		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", idC);
		bb.SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		bb.SetInt("targetId", sortedOpponents.at(2).id);
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = idC;
		rd.tree = "rtt_dennis/MirrorOpponentRole";
		rd.blackboard = bb.toMsg();

		boost::uuids::uuid token = unique_id::fromRandom();
		tokens.push_back(token);
		rd.token = unique_id::toMsg(token);
		pub.publish(rd);
	}

}

bt::Node::Status KickoffDefensePlay::Update() {
	const auto ball = LastWorld::get().ball;
	return ball.pos.x <= -.1 ? Status::Success : Status::Running;
}

}
