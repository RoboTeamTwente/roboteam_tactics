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

#define RTT_CURRENT_DEBUG_TAG StrategyNode

namespace {

std::random_device rd;
std::mt19937 rng(rd());

void feedbackCallback(const roboteam_msgs::RoleFeedbackConstPtr &msg) {
    
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
    rtt::LastRef::set(*refdata);
}

class StrategyDebugInfo {
public:
    StrategyDebugInfo() {
        ros::NodeHandle n;

        strategyInfoPub = n.advertise<roboteam_msgs::StrategyDebugInfo>("strategy_debug_info", 10);
    }


    ros::Publisher strategyInfoPub;

    using Clock = std::chrono::steady_clock;
    using timepoint = std::chrono::steady_clock::time_point;
    using milliseconds = std::chrono::milliseconds;

    timepoint lastDebugMsg = timepoint::min();

    static milliseconds const MSG_INTERVAL;

    bool timeSinceLastMessagePassed() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(Clock::now() - lastDebugMsg) >= MSG_INTERVAL;
    }

    void doUpdate(std::shared_ptr<bt::BehaviorTree> tree) {
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
    ros::init(argc, argv, "StrategyNode");
    ros::NodeHandle n;
    
    namespace f = rtt::factories;

    // Uncomment to check all the captured conditions, skills, tactics, strategies, & roles
    // std::cout << "[Printing everything]\n";
    // f::print_all<rtt::Condition>("Condition");
    // f::print_all<rtt::Skill>("Skill");
    // f::print_all<rtt::Tactic>("Tactic");
    // f::print_all<bt::BehaviorTree>("bt::BehaviorTree");

    ros::Rate rate(60);

    ros::Subscriber feedbackSub = n.subscribe<roboteam_msgs::RoleFeedback>(
        rtt::TOPIC_ROLE_FEEDBACK,
        10,
        &feedbackCallback
        );

    auto directivePub = n.advertise<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE, 100);

    // Construct the global role directive publisher & bt debug publisher if needed
    rtt::GlobalPublisher<roboteam_msgs::RoleDirective> globalRoleDirectivePublisher(rtt::TOPIC_ROLE_DIRECTIVE);
    CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER;
    
    // Creates the callbacks and removes them at the end
    rtt::WorldAndGeomCallbackCreator cb;
    RTT_DEBUGLN("Waiting for first world & geom message...");
    rtt::LastWorld::wait_for_first_messages();

    std::vector<std::string> arguments(argv + 1, argv + argc);

    StrategyDebugInfo stratDebugInfo;

    std::shared_ptr<bt::BehaviorTree> strategy;
    // Only continue if arguments were given
    if (arguments.size() > 0) {
        if (arguments[0] == "mainStrategy") {
            strategy = rtt::StrategyComposer::getMainStrategy();

            std::cout << "mainStrategy: " << strategy << "\n";
            if (!strategy) {
                std::cerr << "There was an error initializing the main strategy tree (StrategyComposer). Aborting.\n";
                return 1;
            }
        } else {
            // Get all available trees
            auto& repo = f::getRepo<f::Factory<bt::BehaviorTree>>();
            // If the given name exists...
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
                ROS_ERROR("\"%s\" is not a strategy tree. Aborting.", arguments[0].c_str());
                return 1;
            }
        }
    } else {
        ROS_ERROR("No strategy tree passed as argument. Aborting.");
        return 1;
    }

    // Wait for all the role nodes to come online if a param was set
    if (rtt::has_PARAM_NUM_ROLE_NODES()) {
        int numNodes;
        rtt::get_PARAM_NUM_ROLE_NODES(numNodes);

        RTT_DEBUGLN("Waiting for %i robot nodes to come online", numNodes);
       
        while ((int) directivePub.getNumSubscribers() < numNodes) {
            RTT_DEBUGLN("Current num subsribers: %i", (int) directivePub.getNumSubscribers());
            ros::spinOnce();
            rate.sleep();

            if (!ros::ok()) {
                RTT_DEBUGLN("Interrupt received, exiting...");
                return 0;
            }
        }
    }

    RTT_DEBUGLN("Found role nodes. Waiting for more than 0 robots to appear...");

    while (rtt::LastWorld::get().us.size() == 0) {
        ros::spinOnce();
        rate.sleep();

        if (!ros::ok()) {
            RTT_DEBUGLN("Interrupt received, exiting...");
            return 0;
        }
    }

    // Possibly initialize based on whatever is present in lastworld, and take the lowest for the keeper?
    // rtt::RobotDealer::initialize_robots(0, {1, 2, 3, 4, 5});

    roboteam_msgs::World world = rtt::LastWorld::get();
    std::vector<int> initializeBots;
    initializeBots.clear();
    for (size_t i = 1; i < world.us.size(); i++) {
        initializeBots.push_back(world.us.at(i).id);
    }
    rtt::RobotDealer::initialize_robots(0, initializeBots);

    RTT_DEBUGLN("More than one robot found. Starting!");
    
    ros::Subscriber ref_sub = n.subscribe<roboteam_msgs::RefereeData>("vision_refbox", 1000, refereeCallback);
    
    while (ros::ok()) {
        ros::spinOnce();

        //bt::Node::Status status =
        bt::Node::Status status = strategy->Update();

        if (status != bt::Node::Status::Running) {
            auto statusStr = bt::statusToString(status);
            RTT_DEBUG("Strategy result: %s. Shutting down...\n", statusStr.c_str());
            break;
        }
        rate.sleep();

        stratDebugInfo.doUpdate(strategy);
    }
    
    // Terminate if needed
    if (strategy->getStatus() == bt::Node::Status::Running) {
        strategy->Terminate(bt::Node::Status::Running);
    }

    return 0;
}
