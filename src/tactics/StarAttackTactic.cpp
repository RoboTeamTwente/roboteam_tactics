#include "roboteam_tactics/tactics/StarAttackTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_msgs/RoleDirective.h"
#include <sstream>
#include "roboteam_tactics/skills/SelectStarAttackShooter.h"

#define RTT_CURRENT_DEBUG_TAG StarAttackTactic

namespace rtt {

RTT_REGISTER_TACTIC(StarAttackTactic);

StarAttackTactic::StarAttackTactic(std::string name, bt::Blackboard::Ptr blackboard)
	: Tactic(name, blackboard),  freeKickTaker(-1), canRun(true) {
	auto geom = LastWorld::get_field();

	for (const Vector2& frac : STAR_POSITION_FRACTIONS) {
		basePositions.push_back({ frac.x * geom.field_length / 2, frac.y * geom.field_width / 2 });
	}
}

void StarAttackTactic::Initialize() {
	static auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
	auto robots = RobotDealer::get_available_robots();

	if (robots.size() < 4) {
		ROS_ERROR("StarAttackTactic: Insufficient robots (%lu)", robots.size());
		canRun = false;
		return;
	}

	freeKickTaker = robots.at(3);

	for (int i = 0; i < 3; i++)
		initBot(robots.at(i), i);

	std::ostringstream ss;
	for (const auto& pair : botParams) {
		ss << pair.first << ",";
	}
	std::string ids = ss.str();


	bt::Blackboard bb;
	bb.SetInt("ROBOT_ID", freeKickTaker);
	bb.SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
	bb.SetString("SelectStarAttackShooter_A_availableIDs", ids);
	bb.SetBool("Kick_A_wait_for_signal", true);
	bb.SetDouble("Kick_A_kickVel", 4.0);
	bb.SetDouble("AimAt_B_xGoal", 3.0);
	roboteam_msgs::RoleDirective rd;
	rd.robot_id = freeKickTaker;
	boost::uuids::uuid token = unique_id::fromRandom();
	botParams[freeKickTaker].token = token;
	rd.token = unique_id::toMsg(token);
	rd.tree = "rtt_dennis/StarAttackFKTRole";
	rd.blackboard = bb.toMsg();
	pub.publish(rd);

	botParams[freeKickTaker] = { Vector2{}, token, rd };
}

bt::Node::Status StarAttackTactic::Update() {
	if (!canRun) {
		return bt::Node::Status::Invalid;
	}

	bool error = false, failure = false, success = true;

    for (const auto& pair : botParams) {
    	if (feedbacks.find(pair.second.token) != feedbacks.end()) {
        	auto stat = feedbacks.at(pair.second.token);
        	error |= stat == bt::Node::Status::Invalid;
        	failure |= stat == bt::Node::Status::Failure;
        	success &= stat == bt::Node::Status::Success;
    	} else {
    		success = false;
    		break;
    	}
    }

    if (error || failure || success) {
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
        for (auto& pair : botParams) {
        	pair.second.rd.tree = pair.second.rd.STOP_EXECUTING_TREE;
        	pub.publish(pair.second.rd);
        }
        return error ? bt::Node::Status::Invalid : (failure ? bt::Node::Status::Failure : bt::Node::Status::Success);
    }

    return bt::Node::Status::Running;
}

double angleToGoal(const Vector2& from) {
	roboteam_msgs::GeometryFieldSize geom = LastWorld::get_field();
	Vector2 goal(-geom.field_length / 2, 0);
	return (goal - from).angle();
}

void StarAttackTactic::initBot(int botID, int posID) {
	static auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
	Vector2 basePos = basePositions.at(posID);
	float xDev = get_rand_real(-DEVIATION_RANGE, DEVIATION_RANGE);
	float yDev = get_rand_real(-DEVIATION_RANGE, DEVIATION_RANGE);
	Vector2 tgtPos = basePos + Vector2(xDev, yDev);
	botParams[botID].tgtPos = tgtPos;
	RTT_DEBUGLN("Bot %d: (%f + %f, %f + %f)", botID, basePos.x, xDev, basePos.y, yDev);

	bt::Blackboard bb;
	bb.SetInt("ROBOT_ID", botID);
	bb.SetInt("KEEPER_ID", GetInt("KEEPER_ID"));

	bb.SetInt("WasIPicked_value", botID);
	bb.SetInt("StandFree_A_theirID", freeKickTaker);
	bb.SetString("StandFree_A_whichTeam", "us");
	bb.SetDouble("StandFree_A_distanceFromPoint", .2);
	bb.SetString("AmITooFarAway_X", "me");
	bb.SetString("AmITooFarAway_Y", "fixed point");
	bb.SetDouble("AmITooFarAway_px", tgtPos.x);
	bb.SetDouble("AmITooFarAway_py", tgtPos.y);
	bb.SetDouble("GoBack_xGoal", tgtPos.x);
	bb.SetDouble("GoBack_yGoal", tgtPos.y);
	bb.SetDouble("GoBack_angleGoal", angleToGoal(tgtPos));
	bb.SetBool("GoBack_avoidRobots", true);

	roboteam_msgs::RoleDirective rd;
	rd.robot_id = botID;
	boost::uuids::uuid token = unique_id::fromRandom();
	botParams[botID].token = token;
	rd.token = unique_id::toMsg(token);
	rd.tree = "rtt_dennis/StarAttackRole";
	rd.blackboard = bb.toMsg();
	pub.publish(rd);
}

}
