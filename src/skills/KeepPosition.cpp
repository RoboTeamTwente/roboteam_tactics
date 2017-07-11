#include "roboteam_tactics/skills/KeepPosition.h"
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

RTT_REGISTER_SKILL(KeepPosition);

KeepPosition::KeepPosition(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {}

bt::Node::Status KeepPosition::Update() {
	if (!gtp) {
		const auto bot = getWorldBot(GetInt("ROBOT_ID"));
		if (!bot) {
			ROS_ERROR("KeepPosition: Bot with ROBOT_ID (=%d) not found...", GetInt("ROBOT_ID"));
			return Status::Invalid;
		}
		if (GetBool("returnToInitialPos", false)) {
			initialPos = Position(*bot);
		}
		private_bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
		private_bb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		private_bb->SetDouble("KeepPosition_GTP_targetAngle", bot->angle);
		private_bb->SetBool("KeepPosition_GTP_avoidRobots", true);
		gtp = std::make_unique<GoToPos>("KeepPosition_GTP", private_bb);
	}
	if (!updateGoalPosition()) {
		return Status::Invalid;
	}
	auto subStatus = gtp->Update();
	if (subStatus == Status::Invalid || subStatus == Status::Failure) {
		return subStatus;
	}
	return Status::Running;
}

bool KeepPosition::updateGoalPosition() {
	const auto bot = getWorldBot(GetInt("ROBOT_ID"));
	if (!bot) {
		ROS_ERROR("KeepPosition: Bot with ROBOT_ID (=%d) not found...", GetInt("ROBOT_ID"));
		return false;
	}

	Vector2 ownPos = initialPos ? initialPos->location() : Vector2{ bot->pos };
	Vector2 nearest = getNearestObject(ownPos);
	Vector2 diff = nearest - ownPos;
	Vector2 goal = diff.length() > MINIMUM_ROBOT_DISTANCE ? ownPos : diff.rotate(M_PI) + ownPos;

	// ROS_INFO_STREAM("ownPos=" << ownPos << " nearest=" << nearest << " diff=" << diff << " goal=" << goal);

	private_bb->SetDouble("KeepPosition_GTP_xGoal", goal.x);
	private_bb->SetDouble("KeepPosition_GTP_yGoal", goal.y);
	private_bb->SetDouble("KeepPosition_GTP_angleGoal", initialPos ? initialPos->rot : bot->angle);
	private_bb->SetDouble("KeepPosition_GTP_maxVelocity", STOP_STATE_MAX_VELOCITY);
	return true;
}

struct DistToPosSorter {
	Vector2 ownPos;
	bool operator()(const Vector2& a, const Vector2& b) {
		return a.dist2(ownPos) < b.dist2(ownPos);
	}
	bool operator()(const roboteam_msgs::WorldRobot& a, const roboteam_msgs::WorldRobot& b) {
		return Vector2{a.pos}.dist2(ownPos) < Vector2{b.pos}.dist2(ownPos);
	}
};

Vector2 KeepPosition::getNearestObject(Vector2 ownPos) {
	const auto& world = LastWorld::get();
	auto us = world.us;
	auto them = world.them;

	std::remove_if(us.begin(), us.end(),
			[=](const roboteam_msgs::WorldRobot& bot) { return bot.id == this->GetInt("ROBOT_ID"); });

	std::sort(us.begin(), us.end(), DistToPosSorter{ownPos});
	std::sort(them.begin(), them.end(), DistToPosSorter{ownPos});

	Vector2 closestUs{us.at(0).pos};
	Vector2 closestThem{them.at(0).pos};
	Vector2 ball{world.ball.pos};

	std::vector<Vector2> v {closestUs, closestThem, ball};
	std::sort(v.begin(), v.end(), DistToPosSorter{ownPos});

	return v.at(0);
}

}
