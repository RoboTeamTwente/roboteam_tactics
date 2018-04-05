#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_tactics/conditions/IsRobotClosestToSecondDangerOpp.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"


#include <string>

namespace rtt {

    RTT_REGISTER_CONDITION(IsRobotClosestToSecondDangerOpp);

    IsRobotClosestToSecondDangerOpp::IsRobotClosestToSecondDangerOpp(std::string name, bt::Blackboard::Ptr blackboard)
            : Condition(name, blackboard) {}




    bt::Node::Status IsRobotClosestToSecondDangerOpp::Update() {

    }

} // rtt
