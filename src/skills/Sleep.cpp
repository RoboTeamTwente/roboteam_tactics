#include <chrono>

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Sleep.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

Sleep::Sleep(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard) {
        	pubSleep = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
}

void Sleep::Initialize() {
    int duration = GetInt("ms");
    start = now();
    std::cout << "Sleeping for approx. " << duration << "ms...\n";
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