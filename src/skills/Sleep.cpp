#include "roboteam_tactics/treegen/LeafRegister.h"
#include <chrono>

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Sleep.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

RTT_REGISTER_SKILL(Sleep);

Sleep::Sleep(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) { }

void Sleep::Initialize() {
    int duration = GetInt("ms");
    start = now();
    // std::cout << "Sleeping for approx. " << duration << "ms...\n";
}

bt::Node::Status Sleep::Update() {
    int duration = GetInt("ms");

    auto elapsedTime = time_difference_milliseconds(start, now());

    if (elapsedTime.count() >= duration) {
        return Status::Success;
    }

    return Status::Running;
}

} // rtt
