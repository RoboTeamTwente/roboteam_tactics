#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/CanSeeRobot.h"
#include "roboteam_tactics/utils/debug_print.h"

#include <vector>
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG CanSeeRobot

namespace rtt {

RTT_REGISTER_CONDITION(CanSeeRobot);

CanSeeRobot::CanSeeRobot(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status CanSeeRobot::Update() {
	RTT_DEBUG("called CanSeeRobot");
	// int robotID = GetInt("ROBOT_ID");
	// ROS_INFO_STREAM("CanSeeRobot Update, id: " << robotID);

	roboteam_msgs::World world = LastWorld::get();
	roboteam_msgs::WorldRobot me = *getWorldBot(blackboard->GetInt("ROBOT_ID"));
    roboteam_msgs::WorldRobot tgt = *getWorldBot(GetInt("targetID"), GetBool("our_team"));
	Vector2 targetPos(tgt.pos);

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
