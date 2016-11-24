#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/CanSeeTheirGoal.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_tactics/utils/debug_print.h"

#include <vector>

#define RTT_CURRENT_DEBUG_TAG CanSeeTheirGoal

namespace rtt {

CanSeeTheirGoal::CanSeeTheirGoal(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status CanSeeTheirGoal::Update() {
	RTT_DEBUG("called CanSeeTheirGoal");
	roboteam_msgs::World world = LastWorld::get();
	auto field = LastWorld::get_field();

	std::string our_field_side;
	n.getParam("our_field_side", our_field_side);

	roboteam_utils::Vector2 goalPos;
	if(our_field_side == "left"){
		goalPos = roboteam_utils::Vector2(field.field_length/2.0, 0);
	} else {
		goalPos = roboteam_utils::Vector2(field.field_length/-2.0, 0);
	}

	auto bb2 = std::make_shared<bt::Blackboard>();
    bb2->SetInt("me", blackboard->GetInt("ROBOT_ID"));
    bb2->SetDouble("x_coor", goalPos.x);
    bb2->SetDouble("y_coor", goalPos.y);
    bb2->SetBool("check_move", true);
	CanSeePoint canSeePoint("", bb2);

	if (canSeePoint.Update() == bt::Node::Status::Success) {
		RTT_DEBUG("i can see the goal!");
		return bt::Node::Status::Success;
	}
	if (canSeePoint.Update() == bt::Node::Status::Failure) {
		RTT_DEBUG("i cannot see the goal :(");
		return bt::Node::Status::Failure;
	}

	return canSeePoint.Update();
}

}
