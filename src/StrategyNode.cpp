#include <algorithm>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "unique_id/unique_id.h"

#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/RoleFeedback.h"
#include "roboteam_msgs/StrategyIgnoreRobot.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/constants.h"
#include "roboteam_msgs/StrategyDebugInfo.h"

#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/generated/alltrees.h"
#include "roboteam_tactics/generated/qualification_trees.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/StrategyComposer.h"
#include "roboteam_tactics/utils/BtDebug.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/BtDebug.h"
#include "roboteam_tactics/utils/RefStateSwitch.h"
#include "roboteam_tactics/utils/CrashHandler.h"

#define RTT_CURRENT_DEBUG_TAG StrategyNode

namespace {

std::random_device rd;
std::mt19937 rng(rd());

void feedbackCallback(const roboteam_msgs::RoleFeedbackConstPtr &msg) {
    ROS_INFO_NAMED("StrategyNode", "");

    auto uuid = unique_id::fromMsg(msg->token);

    if (msg->status == roboteam_msgs::RoleFeedback::STATUS_FAILURE) {
        rtt::feedbacks[uuid] = bt::Node::Status::Failure;
        // std::cout << "Received a feedback on token " << uuid << ": failure.\n";
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_INVALID) {
        rtt::feedbacks[uuid] = bt::Node::Status::Invalid;
        // std::cout << "Received a feedback on token " << uuid << ": invalid.\n";
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_SUCCESS) {
        rtt::feedbacks[uuid] = bt::Node::Status::Success;
        // std::cout << "Received a feedback on token " << uuid << ": success.\n";
    }
}

void refereeCallback(const roboteam_msgs::RefereeDataConstPtr& refdata) {
    ROS_INFO_NAMED("StrategyNode", "Referee command received!");
    rtt::LastRef::set(*refdata);
}

class StrategyDebugInfo {
public:

    using Clock = std::chrono::steady_clock;
    using timepoint = std::chrono::steady_clock::time_point;
    using milliseconds = std::chrono::milliseconds;

    ros::Publisher strategyInfoPub; // Publisher to publish on strategy_debug_info
    timepoint lastDebugMsg = timepoint::min();
    static milliseconds const MSG_INTERVAL;

    StrategyDebugInfo() {
        ROS_INFO_NAMED("StrategyNode", "New instance");

        ros::NodeHandle n;

        strategyInfoPub = n.advertise<roboteam_msgs::StrategyDebugInfo>("strategy_debug_info", 10);

        ROS_INFO_NAMED("StrategyNode", "Publishing on strategy_debug_info");
    }


    bool timeSinceLastMessagePassed() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(Clock::now() - lastDebugMsg) >= MSG_INTERVAL;
    }



    void doUpdate(std::shared_ptr<bt::BehaviorTree> tree) {

        ROS_DEBUG_NAMED("StrategyNode", "Tick");

        auto root = tree->GetRoot();

        if (auto rss = std::dynamic_pointer_cast<rtt::RefStateSwitch>(root)) {
            // It's a refstate switch!
            
            if (rss->hasStartedNewStrategy() || timeSinceLastMessagePassed()) {
                roboteam_msgs::StrategyDebugInfo sdi;

                if (auto refStateOpt = rss->getCurrentRefState()) {
                    sdi.interpreted_ref_command = rtt::refStateToString(*refStateOpt);
                }

                if (auto treeNameOpt = rss->getCurrentStrategyTreeName()) {
                    sdi.current_strategy = *treeNameOpt;
                }

                auto const & robotOwnerMap = rtt::RobotDealer::getRobotOwnerList();
                for (auto const & entry : robotOwnerMap) {
                    // Create a new entry
                    sdi.active_plays.emplace_back();
                    // Get a reference to the new entry
                    auto & playDebugInfo = sdi.active_plays.back();
                    // Fill in the info
                    playDebugInfo.play_name = entry.first;
                    playDebugInfo.active_robots.insert(playDebugInfo.active_robots.end(), entry.second.begin(), entry.second.end());
                }
    
                // Publish it!
                strategyInfoPub.publish(sdi);

                lastDebugMsg = Clock::now();
            }
        } else {
            // Not a refstate switch, so we ignore it.
        }
    }
} ;

StrategyDebugInfo::milliseconds const StrategyDebugInfo::MSG_INTERVAL(200);

}

int main(int argc, char *argv[]) {
    // rtt::crash::registerCrashHandlers();

    ros::init(argc, argv, "StrategyNode");
    ros::NodeHandle n;

    ros::Rate rate(10);

    ros::Subscriber feedbackSub = n.subscribe<roboteam_msgs::RoleFeedback>(
        rtt::TOPIC_ROLE_FEEDBACK,
        10,
        &feedbackCallback
        );

    ROS_INFO_STREAM_NAMED("StrategyNode", "Initializing StrategyNode " << ros::this_node::getName());
    ROS_INFO_NAMED("StrategyNode", "Subscribed to 'role_feedback'");
    // Construct the global role directive publisher & bt debug publisher if needed
    rtt::GlobalPublisher<roboteam_msgs::RoleDirective> globalRoleDirectivePublisher(rtt::TOPIC_ROLE_DIRECTIVE);
    CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER;
    
    // Creates the callbacks and removes them at the end
    rtt::WorldAndGeomCallbackCreator cb;
    ROS_INFO_NAMED("StrategyNode", "Waiting for first world & geom message...");
    rtt::LastWorld::wait_for_first_messages();

    std::vector<std::string> arguments(argv + 1, argv + argc);

    ros::Subscriber ref_sub = n.subscribe<roboteam_msgs::RefereeData>("vision_refbox", 1000, refereeCallback);
    ROS_INFO_NAMED("StrategyNode", "Subscribed to 'vision_refbox'");

    StrategyDebugInfo stratDebugInfo;

    std::shared_ptr<bt::BehaviorTree> strategy;

    // Only continue if arguments were given
    if (arguments.size() > 0) {
        if (arguments[0] == "mainStrategy") {
            strategy = rtt::StrategyComposer::getMainStrategy();

            ROS_INFO_STREAM_NAMED("StrategyNode", "mainStrategy: " << strategy);
            if (!strategy) {
                ROS_ERROR_NAMED("StrategyNode", "There was an error initializing the main strategy tree (StrategyComposer). Aborting.");
                return 1;
            }

            if (!rtt::LastRef::waitForFirstRefCommand()) {
                return 0;
            }
        } else {
            // Get all available trees
            namespace f = rtt::factories;
            auto& repo = f::getRepo<f::Factory<bt::BehaviorTree>>();
            // If the given name exists...



            for (const auto& entry : repo) {
                std::cout << "\t- " << entry.first << "\n";
            }


            if (repo.find(arguments[0]) != repo.end()) {
                // Get the factory
                auto treeFactory = repo.at(arguments[0]);
                // Create the tree
                strategy = treeFactory("", nullptr);

                roboteam_msgs::Blackboard bb;

                RTT_SEND_RQT_BT_TRACE(
                        roboteam_msgs::BtDebugInfo::ID_STRATEGY_NODE, 
                        arguments[0],
                        roboteam_msgs::BtDebugInfo::TYPE_STRATEGY,
                        roboteam_msgs::BtStatus::STARTUP,
                        bb
                        );

            } else {
                ROS_ERROR_NAMED("StrategyNode", "\"%s\" is not a strategy tree. Aborting.", arguments[0].c_str());
                return 1;
            }
        }
    } else {
        ROS_ERROR_NAMED("StrategyNode", "No strategy tree passed as argument. Aborting.");
        return 1;
    }

    // Wait for all the role nodes to come online if a param was set
    if (rtt::has_PARAM_NUM_ROLE_NODES()) {
        int numNodes;
        rtt::get_PARAM_NUM_ROLE_NODES(numNodes);

        ROS_DEBUG_NAMED("StrategyNode", "Waiting for %i robot nodes to come online", numNodes);
       
        auto directivePub = n.advertise<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE, 100);
        while ((int) directivePub.getNumSubscribers() < numNodes) {
            ROS_DEBUG_NAMED("StrategyNode", "Current num subsribers: %i", (int) directivePub.getNumSubscribers());
            ros::spinOnce();
            rate.sleep();

            if (!ros::ok()) {
                RTT_DEBUGLN("Interrupt received, exiting...");
                return 0;
            }
        }
    }

    ROS_DEBUG_NAMED("StrategyNode", "Found role nodes. Waiting for more than 0 robots to appear...");

    while (rtt::LastWorld::get().us.size() == 0) {
        ros::spinOnce();
        rate.sleep();

        if (!ros::ok()) {
            ROS_DEBUG_NAMED("StrategyNode", "Interrupt received, exiting...");
            return 0;
        }
    }
    
    // This is overwritten as soon as the first RefereeCommand comes in, which has the actual keeper robot ID in it.
    rtt::RobotDealer::setKeeper(0);

    ROS_DEBUG_NAMED("StrategyNode", "More than one robot found. Starting!");
    
    while (ros::ok()) {
        ros::spinOnce();

        //bt::Node::Status status =
        bt::Node::Status status = strategy->Update();

        if (status != bt::Node::Status::Running) {
            auto statusStr = bt::statusToString(status);
            ROS_DEBUG_STREAM_NAMED("StrategyNode", "Strategy result: " << statusStr.c_str() << "Shutting down...\n");
            break;
        }
        rate.sleep();

        // Update the keeper according to the ref info
        if (rtt::LastRef::hasReceivedFirstCommand()) {
            rtt::RobotDealer::setKeeper(rtt::LastRef::get().us.goalie);
        }

        stratDebugInfo.doUpdate(strategy);
    }
    
    // Terminate if needed
    if (strategy->getStatus() == bt::Node::Status::Running) {
        strategy->Terminate(bt::Node::Status::Running);
    }

    return 0;
}















