#include <ros/ros.h>

#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_tactics/utils/RefStateSwitch.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/RefLookup.h"
#include "roboteam_tactics/utils/StrategyComposer.h"

#include <boost/optional/optional_io.hpp>


namespace b = boost;

namespace rtt {

    RefStateSwitch::RefStateSwitch() : finishedOnce(false), needToInitialize(false), startedNewStrategy(false), lastKnownBotCount(-1) {}

    void RefStateSwitch::AddStrategy(RefState refState, Node::Ptr child) {
        ROS_DEBUG_STREAM_NAMED("RefStateSwitch", "AddStrategy : " << refStateToString(refState) << " -> " << child->node_name());
        refStateStrategies[refState] = child;
    }



	/**
	 * (re)starts the right strategy tree if needed (change in number of robots, change in referee command)
	 * @return
	 */
    bt::Node::Status RefStateSwitch::Update() {

        // If there is no referee command yet, return RUNNING
        if (!LastRef::hasReceivedFirstCommand()) {
            ROS_WARN_STREAM_NAMED("RefStateSwitch", "Have not yet received a ref command, so not executing any strategy tree.");
            return Status::Running;
        }

        auto world = rtt::LastWorld::get();

        // === If the number of robots change, restart the current strategy === //
        // Get the number of bots in the world
        unsigned botCount = world.us.size();
        // If the amount of bots changed
        if (botCount != lastKnownBotCount) {
            ROS_DEBUG_STREAM_NAMED("RefStateSwitch",
                                   "Botcount changed from " << lastKnownBotCount << " to " << botCount);
            // Update lastKnownBotCount
            lastKnownBotCount = world.us.size();
            // Get the current strategy tree
            if (auto child = getCurrentChild()) {
                // Terminate the strategy tree
                child->Terminate(Status::Invalid);
                // Restart the strategy tree
                child->Initialize();
            }
        }

        startedNewStrategy = false;

        auto cmd = LastRef::getState();

        // === If the referee command changed, then terminate current tree and check for TwoState === //
        if (currentCmd != cmd) {
            ROS_DEBUG_STREAM_NAMED("RefStateSwitch", "RefState switch detected");

            // If there is a new command, terminate the current strategy
            if (currentCmd) {
                getCurrentChild()->Terminate(getCurrentChild()->getStatus()); // getCurrentChild() automatically returns right tree based on previousCmd and currentCmd
            }

            // Update the previous and new command. This causes getCurrentChild() to return a different tree
            previousCmd = currentCmd;
            currentCmd = cmd;

            // Check if the switch is a TwoState switch
            if (isTwoState(previousCmd, *currentCmd)) {
                ROS_DEBUG_STREAM_NAMED("RefStateSwitch", "TwoState detected");
                finishedOnce = false;
            }

            needToInitialize = true;
            startedNewStrategy = true;
        }

        // If the current strategy tree needs to be re-initialized
        if (needToInitialize) {
            needToInitialize = false;

            ROS_DEBUG_STREAM_NAMED("RefStateSwitch", "Initializing current tree : " << getCurrentStrategyTreeName());
            getCurrentChild()->Initialize(); // getCurrentChild() automatically returns right tree based on previousCmd and currentCmd

//        using roboteam_msgs::BtDebugInfo;
//        roboteam_msgs::Blackboard bb;
//        if (auto treeNameOpt = getCurrentStrategyTreeName()) {
//            RTT_SEND_RQT_BT_TRACE(
//                BtDebugInfo::ID_STRATEGY_NODE,
//                *treeNameOpt,
//                BtDebugInfo::TYPE_STRATEGY,
//                roboteam_msgs::BtStatus::STARTUP,
//                bb
//                );
//        }
        }

        // Run the strategy tree
        bt::Node::Status currentStatus = getCurrentChild()->Update();

        if (currentStatus == Status::Failure || currentStatus == Status::Success) {
            ROS_DEBUG_STREAM_NAMED("RefStateSwitch", "Current tree finished : " << getCurrentStrategyTreeName());
            getCurrentChild()->Terminate(currentStatus); // Emiel : Doesn't the tree already terminate on its own? (TODO : Check Node.tick() implementation to confirm)
            needToInitialize = true;
            finishedOnce = true;
        }

        return bt::Node::Status::Running;
    }

    bool RefStateSwitch::hasStartedNewStrategy() const {
        return startedNewStrategy;
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

        // If there was a specific switch from one state to another (isTwoState)
        if (currentCmd && isTwoState(previousCmd, *currentCmd)) {
            // If we are still in the second part of the TwoState
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
