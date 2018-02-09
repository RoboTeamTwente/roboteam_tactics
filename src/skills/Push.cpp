#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/Push.h"

#include "roboteam_msgs/RobotCommand.h"

#define RTT_CURRENT_DEBUG_TAG Push

namespace rtt {

RTT_REGISTER_SKILL(Push);

Push::Push(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        {
            counter = 0;

            if (HasDouble("duration")){
                dur = ceil(60 * GetDouble("duration"));
            } else { dur = ceil(60 * 0.2); }

            if (HasDouble("velocity")){
                vel = GetDouble("velocity");
            } else { vel = 1.0; }

            if (HasDouble("waitDribblerOff")) {
                wait = ceil(60 * GetDouble("waitDribblerOff"));
            } else { wait = 0; }
        }

bt::Node::Status Push::Update() {
    roboteam_msgs::RobotCommand command;
    command.id = blackboard->GetInt("ROBOT_ID");
    // boost::optional<roboteam_msgs::RobotCommand> command = getVelCommand();
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    if (counter < wait + dur) {
        counter++;
        command.dribbler = false;
        if (counter >= wait) {
            command.x_vel = vel;
        }
        pub.publish(command);
        return Status::Running;
    } else {
        command.x_vel = 0.0;
        pub.publish(command);
        return Status::Success;
    }
}

} // rtt