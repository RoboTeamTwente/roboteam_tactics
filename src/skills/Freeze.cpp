#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/skills/Freeze.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

RTT_REGISTER_SKILL(Freeze);
    
Freeze::Freeze(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {}

void Freeze::Initialize() {
    count = 0;
}

bt::Node::Status Freeze::Update() {
    auto pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(stop_command(blackboard->GetInt("ROBOT_ID")));

    int repeat = GetInt("repeat");

    if (repeat == -1) {
        return bt::Node::Status::Running;
    } else {
        auto currentCount = count;
        count += 1;
        if (currentCount >= repeat) {
            return bt::Node::Status::Success;
        } else {
            return bt::Node::Status::Running;
        }
    }

    return count-- == 0 ? bt::Node::Status::Success : bt::Node::Status::Running;
}
    
}
