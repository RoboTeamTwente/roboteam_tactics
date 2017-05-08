#include <ros/ros.h>

#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_tactics/utils/RefStateSwitch.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/RefLookup.h"

namespace rtt {

RefStateSwitch::RefStateSwitch() : validated(false)
                                 , previousCmd(-1)
                                 , currentCmd(-1)
                                 , finishedOnce(false)
                                 , needToInitialize(false) {
                                 // , runningImplicitNormalStartRefCommand(false)
                                 // , switchedToNormal(false) {
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

    // int previousCmd = LastRef::getPreviousRefCommand();
    int cmd = (int) LastRef::get().command.command;

    if (currentCmd != cmd) {
        if (currentCmd != -1) {
            getCurrentChild()->Terminate(getCurrentChild()->getStatus());
        }

        previousCmd = currentCmd;
        currentCmd = cmd;

        if (isTwoStage(previousCmd, currentCmd)) {
            finishedOnce = false;
        }

        needToInitialize = true;

        // if (!isImplicitNormalStartCommand(cmd)) {
            // runningImplicitNormalStartRefCommand = false;
            // hasFinishedTreeOnce = false;
        // } else {
            // if (!runningImplicitNormalStartRefCommand) {
                // runningImplicitNormalStartRefCommand = true;
                // hasFinishedTreeOnce = false;
            // } 
            // // Else we do nothing, we want to retain the state in switchedToNormal and initializedNormal
        // }

        // getCurrentChild()->Initialize();

        // std::cout << "[RefStateSwitch] Current ref command: "
                  // << getRefCommandName(cmd).get_value_or("unknown");
        
        // std::cout << ", previous ref command: "
                  // << getRefCommandName(last).get_value_or("unknown");

        // std::cout << "\n";

        // last = cmd;
    }

    if (needToInitialize) {
        getCurrentChild()->Initialize();
    }

    bt::Node::Status currentStatus = getCurrentChild()->Update();

    if (currentStatus == Status::Failure || currentStatus == Status::Success) {
        getCurrentChild()->Terminate(currentStatus);
        needToInitialize = true;
        finishedOnce = true;
    }

    return bt::Node::Status::Running;
}

bt::Node::Ptr RefStateSwitch::getCurrentChild() {
    if (isTwoStage(previousCmd, currentCmd)) {
        if (finishedOnce) {
            getFirstState(previousCmd, currentCmd);
        } else {
            getSecondState(previousCmd, currentCmd);
        }
    } 
    /* else {
        return children.at(currentCmd);
    } */

    return children.at(currentCmd);

    // if (runningImplicitNormalStartRefCommand && switchedToNormal) {
        // return children.at(roboteam_msgs::RefereeCommand::NORMAL_START);
    // }

    // return children.at(LastRef::get().command.command);
}

// bt::Node::Ptr RefStateSwitch::getPreviousChild() {
    // if (runningImplicitNormalStartRefCommand && switchedToNormal) {
        // return children.at(roboteam_msgs::RefereeCommand::NORMAL_START);
    // }

    // return children.at(last);
// }

void RefStateSwitch::Terminate(Status s) {
    ROS_ERROR_STREAM("TERMINATING THE REF STATE SWITCH IS NOT SUPPORTED!")
    // std::cout << "Terminating RSS!\n";

    // if (last != -1) {
        // if (getCurrentChild()->getStatus() == bt::Node::Status::Running) {
            // getCurrentChild()->Terminate(getCurrentChild()->getStatus());
        // }

        // switchedToNormal = true;

        // last = -1;
    // }

    // // Consider the node failed if it did not properly finish
    // if (s == Status::Running) {
        // setStatus(Status::Failure);
    // }
}

std::string RefStateSwitch::node_name() {
    return "RefStateSwitch";

}

} // rtt
