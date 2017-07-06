#include "roboteam_tactics/skills/DefendPenalty.h"
#include "roboteam_tactics/utils/Intercept.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Section.h"

namespace rtt {

RTT_REGISTER_SKILL(DefendPenalty);

DefendPenalty::DefendPenalty(std::string name, bt::Blackboard::Ptr bb) :
		Skill(name, bb), ballUnderway(false) {
}

bt::Node::Status DefendPenalty::Update() {
	const auto& world = LastWorld::get();
	const auto self = getWorldBot(GetInt("ROBOT_ID"));
	double goalLineX = LastWorld::get_field().field_length / -2;

	if (!ballUnderway) {
		bool isShortRangePenalty = GetBool("isShortRangePenalty", true);
		double penaltyX = LastWorld::get_field().field_length / -2
				+ (isShortRangePenalty ?
						SHORT_RANGE_PENALTY_DIST : LONG_RANGE_PENALTY_DIST);
		double ballX = world.ball.pos.x;

		ballUnderway = ballX < penaltyX - .1;
	}

	if (!gtp) {
		private_bb->SetDouble("gtp_xGoal", goalLineX);
		private_bb->SetDouble("gtp_yGoal", 0);
		private_bb->SetDouble("gtp_angleGoal", 0);
		private_bb->SetBool("gtp_enterDefenseAreas", true);
		gtp = std::make_unique<GoToPos>("gtp", private_bb);
	}

	if (ballUnderway) {
		auto query = buildNormalInterceptQuery(Position { *self }, self->vel,
				world.ball.pos, world.ball.vel);
		auto result = calculateInterception(query);
		Vector2 tgtVector;
		if (!result.success) {
			ROS_WARN(
					"DefendPenalty: ballUnderway is true but calculateInterception failed to find an interception!");
			tgtVector = Vector2(world.ball.pos) + Vector2(world.ball.vel) / 5;
		} else {
			tgtVector = Vector2(result.pos);
		}
		private_bb->SetDouble("gtp_xGoal", tgtVector.x);
		private_bb->SetDouble("gtp_yGoal", tgtVector.y);
		gtp->Update();
		return Status::Running;
	}
	auto ballHolder = getBallHolder();

	Vector2 tgtVector;
	if (ballHolder) {
		Position shooterPos(ballHolder->first);

		Section goalSection { goalLineX, 0.5, goalLineX, -.5 };

		Vector2 shooterAimVector = (shooterPos.location() + Vector2{10, 0}).rotate(
				shooterPos.location().angle());
		Section shooterAimSection { shooterPos.location(), shooterAimVector };
		Vector2 intersection = goalSection.intersection(shooterAimSection);
		ROS_INFO_STREAM("Shooter Aim: " << shooterAimSection.a << " -- " << shooterAimSection.b);
		ROS_INFO_STREAM("Intersection: " << intersection);
		if (goalSection.pointOnLine(intersection)) {
			tgtVector = intersection;
		} else {
			tgtVector = { goalLineX, 0 };
		}
	} else {
		tgtVector = { goalLineX, 0 };
	}

	private_bb->SetDouble("gtp_xGoal", tgtVector.x);
	private_bb->SetDouble("gtp_yGoal", tgtVector.y);
	gtp->Update();
	return Status::Running;
}

}
