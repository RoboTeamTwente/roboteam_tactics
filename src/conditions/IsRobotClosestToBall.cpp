#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/IsRobotClosestToBall.h"

#include <vector>
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(IsRobotClosestToBall);

IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status IsRobotClosestToBall::Update() {

	roboteam_msgs::World world = LastWorld::get();
	int robotID = GetInt("ROBOT_ID");
	Vector2 ballPos(world.ball.pos);
	std::vector<roboteam_msgs::WorldRobot> robots = world.us;
	boost::optional<int> robotClosestToBallPtr = get_robot_closest_to_point(robots, ballPos);
	if (robotClosestToBallPtr) {
		int robotClosestToBall = *robotClosestToBallPtr;
		if (robotID == robotClosestToBall) {
			return Status::Success;
		} else {
			return Status::Failure;
		}
	} else {
		return Status::Failure;
	}
}

}
