#include "roboteam_tactics/skills/KickPenalty.h"
#include "roboteam_msgs/RobotCommand.h"

namespace rtt {

RTT_REGISTER_SKILL(KickPenalty);

KickPenalty::KickPenalty(std::string, bt::Blackboard::Ptr bb) : Skill(name, bb),
		choseTarget{false}, shootingAtTopCorner{false}, rotateDone{false}, kickCount{0} {}

boost::optional<roboteam_msgs::WorldRobot> findKeeper() {
	const auto& world = LastWorld::get();
	for (const auto& bot : world.them) {
		if (bot.pos.x > 4 && fabs(bot.pos.y) < 2) {
			return bot;
		}
	}
	return boost::none;
}

bt::Node::Status KickPenalty::Update() {
	if (!choseTarget) {
		choseTarget = true;
		if (HasString("targetCorner")) {
			shootingAtTopCorner = GetString("targetCorner") == "top";
		} else {
			auto keeper = findKeeper();
			if (keeper) {
				shootingAtTopCorner = keeper->pos.y < 0;
			} else {
				ROS_WARN("KickPenalty: Could not determine who the keeper is, choosing a corner at random.");
				shootingAtTopCorner = get_rand_int(1) == 0;
			}
		}
	}

	if (!rotateDone) {
		if (!rotator) {
			private_bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
			private_bb->SetString("RAP_center", "ball");

			auto geom = LastWorld::get_field();
			double x = geom.field_length / 2;
			double y = shootingAtTopCorner
						?  geom.goal_width / 2 - AIM_SAFETY_MARGIN
						: -geom.goal_width / 2 + AIM_SAFETY_MARGIN;
			private_bb->SetDouble("RAP_faceTowardsPosx", x);
			private_bb->SetDouble("RAP_faceTowardsPosy", y);
			private_bb->SetDouble("RAP_w", 2.5);
			private_bb->SetDouble("RAP_maxv", 2.5); // Default is 1, but we need to be as fast as possible
			rotator = std::make_unique<RotateAroundPoint>("RAP", private_bb);
		}
		auto rotateStatus = rotator->Update();
		switch (rotateStatus) {
		case Status::Failure:
		case Status::Invalid:
			return rotateStatus;
		case Status::Success:
			rotateDone = true;
			break;
		default:
			return Status::Running;
		}
	}

	auto& pub = GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
	roboteam_msgs::RobotCommand cmd;
	cmd.id = GetInt("ROBOT_ID");
	cmd.x_vel = 0;
	cmd.y_vel = 0;
	cmd.w = 0;
	cmd.kicker = true;
	cmd.kicker_forced = true;
	cmd.kicker_vel = 7.5;
	cmd.chipper = false;
	cmd.chipper_forced = false;
	cmd.chipper_vel = 0;
	cmd.dribbler = false;
	cmd.active = true;
	pub.publish(cmd);
	return kickCount++ < KICK_COUNT ? Status::Running : Status::Success;
}

}
