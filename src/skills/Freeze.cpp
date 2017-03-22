#include "roboteam_tactics/skills/Freeze.h"

#include "roboteam_tactics/utils/utils.h"

namespace rtt {
    
Freeze::Freeze(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {}

bt::Node::Status Freeze::Update() {
    static auto init = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS);
    static auto pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(stop_command(blackboard->GetInt("ROBOT_ID")));
    return count-- == 0 ? bt::Node::Status::Success : bt::Node::Status::Running;
}
    
}