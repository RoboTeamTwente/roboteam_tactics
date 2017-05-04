#include <ros/ros.h>

#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_tactics/utils/RefStateSwitch.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/RefLookup.h"

namespace rtt {

RefStateSwitch::RefStateSwitch() : validated(false)
                                 , last(-1)
                                 , runningImplicitNormalStartRefCommand(false)
                                 , switchedToNormal(false) {
    std::cout << "Initializing last!\n";
}

bool RefStateSwitch::isValid() const {
    return children.size() == REF_STATE_COUNT;
}

void RefStateSwitch::assertValid() const {
    if (!isValid()) {
        ROS_ERROR("Assertion Failed: RefStateSwitch expected %lu children, but got %lu.",
                    REF_STATE_COUNT, children.size());
    }
}

void RefStateSwitch::AddChild(Node::Ptr child) {
    if (!validated) {
        bt::Composite::AddChild(child);
    }
}

bt::Node::Status RefStateSwitch::Update() {
    if (!validated) {
        assertValid();
    }

    validated = true;

    int cmd = (int) LastRef::get().command.command;

    if (last != cmd) {
        if (last != -1) {
            getPreviousChild()->Terminate(getPreviousChild()->getStatus());
        }

        if (!isImplicitNormalStartCommand(cmd)) {
            runningImplicitNormalStartRefCommand = false;
            switchedToNormal = false;
        } else {
            if (!runningImplicitNormalStartRefCommand) {
                runningImplicitNormalStartRefCommand = true;
                switchedToNormal = false;
            } else {
                // Do nothing, we want to retain the state in switchedToNormal and initializedNormal
            }
        }

        getCurrentChild()->Initialize();

        std::cout << "[RefStateSwitch] Current ref command: "
                  << getRefCommandName(cmd).get_value_or("unknown");
        
        std::cout << ", previous ref command: "
                  << getRefCommandName(last).get_value_or("unknown");

        std::cout << "\n";

        last = cmd;
    }

    return getCurrentChild()->Update();
}

bt::Node::Ptr RefStateSwitch::getCurrentChild() {
    if (runningImplicitNormalStartRefCommand && switchedToNormal) {
        return children.at(roboteam_msgs::RefereeCommand::NORMAL_START);
    }

    return children.at(LastRef::get().command.command);
}

bt::Node::Ptr RefStateSwitch::getPreviousChild() {
    if (runningImplicitNormalStartRefCommand && switchedToNormal) {
        return children.at(roboteam_msgs::RefereeCommand::NORMAL_START);
    }

    return children.at(last);
}

void RefStateSwitch::Terminate(Status s) {
    std::cout << "Terminating RSS!\n";

    if (last != -1) {
        if (getCurrentChild()->getStatus() == bt::Node::Status::Running) {
            getCurrentChild()->Terminate(getCurrentChild()->getStatus());
        }

        switchedToNormal = true;

        last = -1;
    }

    // Consider the node failed if it did not properly finish
    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}

std::string RefStateSwitch::node_name() {
    return "RefStateSwitch";

}

} // rtt
