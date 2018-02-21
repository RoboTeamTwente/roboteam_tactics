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


// Tweaked by Jelle to use Jelle's variant of getting distance to defense area, which now returns a double instead of a vector.
bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point, double margin) {
    GeometryFieldSize field = LastWorld::get_field();
    double distToDefenseArea = getDistToDefenseArea2(ourDefenseArea, point);
    if (distToDefenseArea < margin) return true;
    else return false;
}

// bool isWithinDefenseArea(bool ourDefenseArea, Vector2 point, double safetyMargin) {
//     GeometryFieldSize field = LastWorld::get_field();
//     Vector2 distToDefenseArea = getDistToDefenseArea(ourDefenseArea, point, safetyMargin);
//     if (ourDefenseArea) {
//         if (distToDefenseArea.x > 0.0 && point.x >= -field.field_length/2) return true;
//         else return false;
//     } else {
//         if (distToDefenseArea.x < 0.0 && point.x <= field.field_length/2) return true;
//         else return false;
//     }
// }




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

	double margin = 0.0;
	if (HasDouble("margin")) {
		margin = GetDouble("margin");
	}

	Vector2 point = ballPos;

	// THE CONDITION NEEDS DIFFERENT NAME, BECAUSE WE CAN PUT IN ROBOT POS AS WELL NOW

	// If robot pos should be checked instead of ball pos, get my position and use that as the point.
	if (HasBool("robot") && GetBool("robot")){
		int robotID = GetInt("ROBOT_ID");
		boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    	roboteam_msgs::WorldRobot me;
    	if (findBot) {
	        me = *findBot;
	    } else {
        	ROS_WARN_STREAM("GoToPos: robot with this ID not found, ID: " << robotID);
        	return Status::Invalid;
    	}
		point = me.pos;
	}

	// Do the check
	if (isWithinDefenseArea(ourDefenseArea, point, margin)) {
		return Status::Success;
	} else {
		return Status::Failure;
	}
}

} // rtt
