#include "roboteam_tactics/treegen/LeafRegister.h"
#include <chrono>

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

void DebugTrace::Initialize() {
}

bt::Node::Status DebugTrace::Update() {
    std::cout << "Update from " << name << "\n";

    return Status::Running;
}

} // rtt

