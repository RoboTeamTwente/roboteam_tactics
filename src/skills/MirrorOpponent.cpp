#include "roboteam_tactics/skills/MirrorOpponent.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

RTT_REGISTER_SKILL(MirrorOpponent);

MirrorOpponent::MirrorOpponent(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {}

bt::Node::Status MirrorOpponent::Update() {
	int tgtId = GetInt("targetId");
	const auto tgt = getWorldBot(tgtId, false);
	if (!tgt) {
		ROS_WARN("MirrorOpponent (ROBOT_ID=%d): Opponent with id %d not found...", GetInt("ROBOT_ID"), tgtId);
		return Status::Failure;
	}

	Vector2 tgtPos(tgt->pos);
	double mirrorX = GetDouble("mirrorX", 0.0);
	if (tgtPos.x <= mirrorX) {
		ROS_WARN("MirrorOpponent (ROBOT_ID=%d): Opponent %d has passed the mirror line (%f)",
				GetInt("ROBOT_ID"), tgtId, mirrorX);
		return Status::Failure;
	}

	double xGoal;
	if (GetBool("alongMirror", true)) {
		xGoal = mirrorX - .15;
	} else {
		xGoal = mirrorX - fabs(mirrorX - tgtPos.x);
	}

	Vector2 goalPos { xGoal, tgtPos.y };
	double goalAngle = (tgtPos - goalPos).angle();

	ROS_INFO("Mirror goal for %d: (%f, %f, %f)", GetInt("ROBOT_ID"), goalPos.x, goalPos.y, goalAngle);

	private_bb->SetDouble("MirrorOpponent_GoToPos_xGoal", goalPos.x);
	private_bb->SetDouble("MirrorOpponent_GoToPos_yGoal", goalPos.y);
	private_bb->SetDouble("MirrorOpponent_GoToPos_angleGoal", goalAngle);

	if (!gtp) {
		private_bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
		gtp = std::make_unique<GoToPos>("MirrorOpponent_GoToPos", private_bb);
		gtp->Initialize();
	}
	auto gtpResult = gtp->Update();
	if (gtpResult == Status::Failure || gtpResult == Status::Invalid) {
		return gtpResult;
	}
	return Status::Running;
}

}
