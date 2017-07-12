#include <ros/ros.h>

#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_tactics/utils/RefStateSwitch.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/RefLookup.h"
#include "roboteam_tactics/utils/StrategyComposer.h"

namespace b = boost;

namespace rtt {

RefStateSwitch::RefStateSwitch() : validated(false)
                                 // , previousCmd(-1)
                                 // , currentCmd(-1)
                                 , finishedOnce(false)
                                 , needToInitialize(false)
                                 , startedNewStrategy(false) {
                                 // , runningImplicitNormalStartRefCommand(false)
                                 // , switchedToNormal(false) {
}

// TODO: Reimplement this
bool RefStateSwitch::isValid() const {
    // return children.size() == REF_STATE_COUNT;
    return true;
}

// TODO: Reimplement this
void RefStateSwitch::assertValid() const {
    if (!isValid()) {
        // ROS_ERROR("Assertion Failed: RefStateSwitch expected %lu children, but got %lu.",
                    // REF_STATE_COUNT, children.size());
        ROS_ERROR("Something went wrong in RefStateSwitch!");
    }
}

void RefStateSwitch::AddStrategy(RefState refState, Node::Ptr child) {
    refStateStrategies[refState] = child;
}

bt::Node::Status RefStateSwitch::Update() {
    if (!validated) {
        assertValid();
        validated = true;
    }

    if (!LastRef::hasReceivedFirstCommand()) {
        ROS_WARN_STREAM("Have not yet received a ref command, so not executing any strategy tree.");
        return Status::Running;
    }

    startedNewStrategy = false;

    auto cmd = LastRef::getState();

    if (currentCmd != cmd) {
        if (currentCmd) {
            getCurrentChild()->Terminate(getCurrentChild()->getStatus());
        }

        previousCmd = currentCmd;
        currentCmd = cmd;

        if (isTwoState(previousCmd, *currentCmd)) {
            finishedOnce = false;
        }

        needToInitialize = true;
        startedNewStrategy = true;
    }

    if (needToInitialize) {
        needToInitialize = false;

        getCurrentChild()->Initialize();

        using roboteam_msgs::BtDebugInfo;
        roboteam_msgs::Blackboard bb;
        if (auto treeNameOpt = getCurrentStrategyTreeName()) {
            RTT_SEND_RQT_BT_TRACE(
                    BtDebugInfo::ID_STRATEGY_NODE,
                    *treeNameOpt,
                    BtDebugInfo::TYPE_STRATEGY,
                    roboteam_msgs::BtStatus::STARTUP,
                    bb
                    );
        }
    }

    bt::Node::Status currentStatus = getCurrentChild()->Update();

    if (currentStatus == Status::Failure || currentStatus == Status::Success) {
        getCurrentChild()->Terminate(currentStatus);
        needToInitialize = true;
        finishedOnce = true;
    }

    return bt::Node::Status::Running;
}

b::optional<RefState> RefStateSwitch::getCurrentRefState() const {
    std::string previousCmdName = "none yet";
    std::string currentCmdName = "none yet";

    if (currentCmd) {
        currentCmdName = refStateToString(*currentCmd);
    }

    if (previousCmd) {
        previousCmdName = refStateToString(*previousCmd);
    }

    if (currentCmd && isTwoState(previousCmd, *currentCmd)) {
        if (!finishedOnce) {
            if (auto targetStateOpt = getFirstState(previousCmd, *currentCmd)) {
                return *targetStateOpt;
            } else {
                ROS_ERROR("PreviousCmd and currentCmd are twoState states, but getFirstState "
                          "returned nothing! PreviousCmd: %s, currentCmd: %s",
                          previousCmdName.c_str(),
                          currentCmdName.c_str()
                          );

                return b::none;
            }
        } else {
            // Second state of a twostate pair is always normal play
            return RefState::NORMAL_START;
        }
    } else if (!currentCmd) {
        return b::none;
    } else {
        return *currentCmd;
    }
}

b::optional<std::string> RefStateSwitch::getCurrentStrategyTreeName() const {
    if (auto refStateOpt = getCurrentRefState()) {
        auto const it = StrategyComposer::MAPPING.find(*refStateOpt);
        if (it != StrategyComposer::MAPPING.end()) {
            return it->second;
        }
    }

    // This is not an error, because if there is a none in the
    // strategy composer then it should just run the
    // normal play in that case
    return b::none;
}

bt::Node::Ptr RefStateSwitch::getCurrentChild() {
    std::string previousCmdName = "none yet";
    std::string currentCmdName = "none yet";

    if (currentCmd) {
        currentCmdName = refStateToString(*currentCmd);
    }

    if (previousCmd) {
        previousCmdName = refStateToString(*previousCmd);
    }

    if (auto targetStateOpt = getCurrentRefState()) {
        auto stateIt = refStateStrategies.find(*targetStateOpt);
        if (stateIt != refStateStrategies.end()) {
            return stateIt->second;
        } else {
            ROS_ERROR("No strategy tree found! Previouscmd: %s, currentCmd: %s",
                    previousCmdName.c_str(),
                    currentCmdName.c_str()
                    );
            
            return nullptr;
        }
    } else {
        return nullptr;
    }
}

void RefStateSwitch::Terminate(Status) {
    ROS_ERROR_STREAM("TERMINATING THE REF STATE SWITCH IS NOT SUPPORTED!");
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
