#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/CanSeeRobot.h"
#include "roboteam_tactics/utils/debug_print.h"

#include <vector>

#define RTT_CURRENT_DEBUG_TAG CanSeeRobot

namespace rtt {

CanSeeRobot::CanSeeRobot(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status CanSeeRobot::Update() {
	RTT_DEBUG("called CanSeeRobot");

	roboteam_msgs::World world = LastWorld::get();
	roboteam_msgs::WorldRobot me = world.us.at(blackboard->GetInt("ROBOT_ID"));
	roboteam_utils::Vector2 targetPos;
	if (GetBool("our_team")) {
		targetPos = roboteam_utils::Vector2(world.us.at(GetInt("targetID")).pos.x, world.us.at(GetInt("targetID")).pos.y);
	} else {
		targetPos = roboteam_utils::Vector2(world.them.at(GetInt("targetID")).pos.x, world.them.at(GetInt("targetID")).pos.y);
	}

	std::vector<roboteam_msgs::WorldRobot> obstacles = getObstacles(me, targetPos, &world, true);
	if (obstacles.size() > 1) {
		RTT_DEBUG("i cannot see the robot :(");
		return Status::Failure;
	} else {
		RTT_DEBUG("i can see the robot!");
		return Status::Success;
	}
}

}
