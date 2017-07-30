#include "roboteam_tactics/conditions/CanKeeperMove.h"

namespace rtt {

RTT_REGISTER_CONDITION(CanKeeperMove);

CanKeeperMove::CanKeeperMove(std::string name, bt::Blackboard::Ptr bb) : Condition(name, bb) {}

bt::Node::Status CanKeeperMove::Update() {
	bool isShootout = LastRef::get().stage.stage
			== roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT;
	bool hasBegun = LastRef::getState() == RefState::NORMAL_START;
	ROS_INFO("CanKeeperMove: isShootout=%d, hasBegun=%d", isShootout, hasBegun);
	if (isShootout) {
		return hasBegun ? Status::Success : Status::Failure;
	}
	if (hasBegun) {
		if (!initialBallPos) {
			initialBallPos = LastWorld::get().ball.pos;
		}
		Vector2 currentBallPos = LastWorld::get().ball.pos;
		ROS_INFO_STREAM("initial: " << *initialBallPos << ", current: "
				<< currentBallPos << " dist: " << currentBallPos.dist(*initialBallPos));
		if (currentBallPos.dist(*initialBallPos) > .02) {
			return Status::Success;
		}
		return Status::Failure;
	} else {
		return Status::Failure;
	}
}

}
