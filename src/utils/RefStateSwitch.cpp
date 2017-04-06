#include <ros/ros.h>

#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_tactics/utils/RefStateSwitch.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/RefLookup.h"

namespace rtt {

RefStateSwitch::RefStateSwitch() : validated(false), last(-1) {
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

    auto child = children.at(cmd);

    if (last != cmd) {
        if (last != -1) {
            children.at(last)->Terminate(children.at(last)->getStatus());
        }

        child->Initialize();

        std::cout << "[RefStateSwitch] Current ref command: "
                  << getRefCommandName(cmd).get_value_or("unknown");
        
        std::cout << ", previous ref command: "
                  << getRefCommandName(last).get_value_or("unknown");

        std::cout << "\n";

        last = cmd;
    }

    return child->Update();
}

void RefStateSwitch::Terminate(Status s) {
    std::cout << "Terminating RSS!\n";

    if (last != -1) {
        auto& child = children.at(last);

        // Only terminate if the child did not do so properly itself
        if (child->getStatus() == bt::Node::Status::Running) {
            child->Terminate(bt::Node::Status::Running);
        }

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
