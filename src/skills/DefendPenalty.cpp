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

bool ballIsFarEnoughAway(Vector2 ballPos, double penaltyX) {
	if (ballPos.x > penaltyX + 1) return true;
	double goalX = LastWorld::get_field().field_length / -2;
	return ballPos.dist({goalX, 0}) > 1.5;
}

bt::Node::Status DefendPenalty::Update() {
	const auto& world = LastWorld::get();
	const auto self = getWorldBot(GetInt("ROBOT_ID"));
	double goalLineX = LastWorld::get_field().field_length / -2;
	bool isShortRangePenalty = GetBool("isShortRangePenalty", true);
	double penaltyX = LastWorld::get_field().field_length / -2
			+ (isShortRangePenalty ?
					SHORT_RANGE_PENALTY_DIST : LONG_RANGE_PENALTY_DIST);

	if (!ballUnderway) {
		double ballX = world.ball.pos.x;
		ballUnderway = ballX < penaltyX - .1;
	}

	if (!gtp) {
		private_bb->SetDouble("gtp_xGoal", goalLineX);
		private_bb->SetDouble("gtp_yGoal", 0);
		private_bb->SetDouble("gtp_angleGoal", M_PI_2); // Rotate the robot so it faces one side of the goal
		                                                // since this will help acceleration along the goal line.
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
		return ballIsFarEnoughAway(world.ball.pos, penaltyX) ? Status::Success : Status::Running;
	}
	auto ballHolder = getBallHolder();

	Vector2 tgtVector;
	if (ballHolder) {
		Position shooterPos(ballHolder->first);

		Section goalSection { goalLineX, 0.5, goalLineX, -.5 };

		Vector2 shooterAimVector = (shooterPos.location() + Vector2{10, 0}).rotate(
				-shooterPos.location().angle());
		Section shooterAimSection { shooterPos.location(), shooterAimVector };
		Vector2 intersection = goalSection.intersection(shooterAimSection);
		if (goalSection.pointOnLine(intersection)) {
			tgtVector = intersection;
		} else {
			tgtVector = { goalLineX, 0 };
		}
	} else {
		tgtVector = { goalLineX, 0 };
	}

	private_bb->SetDouble("gtp_xGoal", tgtVector.x);
	private_bb->SetDouble("gtp_yGoal", -tgtVector.y);
	gtp->Update();
	return Status::Running;
}

}
