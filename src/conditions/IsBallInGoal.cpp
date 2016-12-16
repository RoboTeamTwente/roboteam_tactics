#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/IsBallInGoal.h"

#include <vector>
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(IsBallInGoal);

IsBallInGoal::IsBallInGoal(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status IsBallInGoal::Update() {
	// ROS_INFO("called IsBallInGoal");
	roboteam_msgs::World world = LastWorld::get();
	auto field = LastWorld::get_field();
	roboteam_utils::Vector2 ballPos(world.ball.pos.x, world.ball.pos.y);
	std::string our_side = get_our_side();

	if (GetBool("our_goal")) {
		if(our_side == "left"){
			roboteam_utils::Vector2 goalPos = roboteam_utils::Vector2(field.field_length/-2.0, 0);
			if (ballPos.x <= goalPos.x && ballPos.x >= (goalPos.x-field.goal_depth) && abs(ballPos.y) <= field.goal_width) return Status::Success;
		} else {
			roboteam_utils::Vector2 goalPos = roboteam_utils::Vector2(field.field_length/2.0, 0);
			if (ballPos.x >= goalPos.x && ballPos.x <= (goalPos.x+field.goal_depth) && abs(ballPos.y) <= field.goal_width) return Status::Success;
		}
	} else {
		if(our_side == "left"){
			roboteam_utils::Vector2 goalPos = roboteam_utils::Vector2(field.field_length/2.0, 0);
			if (ballPos.x >= goalPos.x && ballPos.x <= (goalPos.x+field.goal_depth) && abs(ballPos.y) <= field.goal_width) return Status::Success;
		} else {
			roboteam_utils::Vector2 goalPos = roboteam_utils::Vector2(field.field_length/-2.0, 0);
			if (ballPos.x <= goalPos.x && ballPos.x >= (goalPos.x-field.goal_depth) && abs(ballPos.y) <= field.goal_width) return Status::Success;
		}
	}
	return Status::Failure;
}

}
