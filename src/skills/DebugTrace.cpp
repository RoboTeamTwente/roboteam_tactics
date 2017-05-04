#include "roboteam_tactics/treegen/LeafRegister.h"
#include <chrono>
#include <string>

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/DebugTrace.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

RTT_REGISTER_SKILL(DebugTrace);

DebugTrace::DebugTrace(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) { }

void DebugTrace::Initialize() { }

bt::Node::Status DebugTrace::Update() {
    if (HasString("msg")) {
        std::cout << "[DebugTrace] " << name << ": " << GetString("msg") << "\n";
    } else {
        std::cout << "[DebugTrace] " << name << "\n";
    }

    std::string doReturn = GetString("doReturn");

    if (doReturn=="Success") {
    	return Status::Success;
    } else if (doReturn=="Failure") {
    	return Status::Failure;
    }

    return Status::Running;
}

} // rtt

