#include <algorithm>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/RoleFeedback.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/StrategyIgnoreRobot.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/generated/alltrees.h"
#include "roboteam_tactics/generated/alltrees_list.h"
#include "roboteam_tactics/generated/alltrees_factory.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG StrategyNode

std::random_device rd;
std::mt19937 rng(rd());

void feedbackCallback(const roboteam_msgs::RoleFeedbackConstPtr &msg) {
    auto uuid = unique_id::fromMsg(msg->token);

    if (msg->status == roboteam_msgs::RoleFeedback::STATUS_FAILURE) {
        rtt::feedbacks[uuid] = bt::Node::Status::Failure;
        std::cout << "Received a feedback on token " << uuid << ": failure.\n";
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_INVALID) {
        rtt::feedbacks[uuid] = bt::Node::Status::Invalid;
        std::cout << "Received a feedback on token " << uuid << ": invalid.\n";
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_SUCCESS) {
        rtt::feedbacks[uuid] = bt::Node::Status::Success;
        std::cout << "Received a feedback on token " << uuid << ": success.\n";
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "StrategyNode");
    ros::NodeHandle n;

    ros::Rate rate(60);

    auto directivePub = n.advertise<roboteam_msgs::RoleDirective>("role_directive", 100);
    
    RTT_CREATE_WORLD_AND_GEOM_CALLBACKS;
    rtt::LastWorld::wait_for_first_messages();

    std::vector<std::string> arguments(argv + 1, argv + argc);

    bt::BehaviorTree strategy;
    if (arguments.size() > 0) {
        strategy = rtt::make_tree(arguments[0], n);
    } else {
        ROS_ERROR("No strategy tree passed as argument. Aborting.");
        return 1;
    }

    // Wait for all the role nodes to come online if a param was set
    if (ros::param::has("num_role_nodes")) {
        int numNodes;
        ros::param::get("num_role_nodes", numNodes);
        RTT_DEBUG("Waiting for %d role nodes...\n", numNodes);
        while ((int) directivePub.getNumSubscribers() < numNodes) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    RTT_DEBUG("Found role nodes. Waiting for more than 0 robots to appear...\n");

    while (rtt::LastWorld::get().us.size() == 0) {
        ros::spinOnce();
        rate.sleep();
    }

    // Possibly initialize based on whatever is present in lastworld, and take the lowest for the keeper?
    rtt::RobotDealer::initialize_robots(0, {1, 2, 3, 4, 5});

    RTT_DEBUG("More than one robot found. Starting...\n");

    while (ros::ok()) {
        ros::spinOnce();

        bt::Node::Status status = strategy.Update();

        if (status != bt::Node::Status::Running) {
            auto statusStr = bt::statusToString(status);
            RTT_DEBUG("Strategy result: %s. Starting again.\n", statusStr.c_str());
        }

        rate.sleep();
    }

    return 0;
}
