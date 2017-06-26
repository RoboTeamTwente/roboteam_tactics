#include <vector>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/CanSeeTheirGoal.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/ComputePassPoint.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG CanSeeTheirGoal

namespace rtt {

RTT_REGISTER_CONDITION(CanSeeTheirGoal);

CanSeeTheirGoal::CanSeeTheirGoal(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status CanSeeTheirGoal::Update() {
	RTT_DEBUG("called CanSeeTheirGoal");
	roboteam_msgs::World world = LastWorld::get();

	int robotID = blackboard->GetInt("ROBOT_ID");
	boost::optional<roboteam_msgs::WorldRobot> robotPointer = getWorldBot(robotID);
    roboteam_msgs::WorldRobot robot;
    if (robotPointer) {
        robot = *robotPointer;
    } else {
        ROS_WARN("Kick: Robot not found");
        return Status::Failure;
    }
    Vector2 robotPos(robot.pos);
    PassPoint passPoint;
	double viewOfGoal = passPoint.calcViewOfGoal(robotPos, world);

	if (viewOfGoal >= 0.3) {
		return Status::Success;
	} else {
		return Status::Failure;
	}
}

} // rtt