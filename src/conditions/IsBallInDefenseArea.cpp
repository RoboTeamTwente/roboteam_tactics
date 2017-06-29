#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/IsBallInDefenseArea.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"

#include <vector>
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point) {
    GeometryFieldSize field = LastWorld::get_field();
    Vector2 distToDefenseArea = getDistToDefenseArea(ourDefenseArea, point, 0.0);
    if (ourDefenseArea) {
        if (distToDefenseArea.x > 0.0 && point.x >= -field.field_length/2) return true;
        else return false;
    } else {
        if (distToDefenseArea.x < 0.0 && point.x <= field.field_length/2) return true;
        else return false;
    }
}


RTT_REGISTER_CONDITION(IsBallInDefenseArea);

IsBallInDefenseArea::IsBallInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {}

bt::Node::Status IsBallInDefenseArea::Update() {
	
	bool ourDefenseArea;
	if (HasBool("ourDefenseArea")) {
		ourDefenseArea = GetBool("ourDefenseArea");
	} else {
		ourDefenseArea = true;
	}

	roboteam_msgs::World world = LastWorld::get();
	Vector2 ballPos(world.ball.pos);

	if (isWithinDefenseArea(ourDefenseArea, ballPos)) {
		return Status::Success;
	} else {
		return Status::Failure;
	}
}

} // rtt