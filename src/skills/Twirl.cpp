#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Twirl.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_SKILL(Twirl);
    
Twirl::Twirl(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {}

bt::Node::Status Twirl::Update() {
    roboteam_msgs::RobotCommand rc;

    rc.id = blackboard->GetInt("ROBOT_ID");

    if (HasDouble("speed")) {
        rc.w = GetDouble("speed");
    } else {
        rc.w = 0.5;
    }

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(rc);

    return bt::Node::Status::Running;
}
    
}
