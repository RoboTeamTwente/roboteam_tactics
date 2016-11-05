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
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/generated/alltrees.h"
#include "roboteam_tactics/generated/alltrees_list.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/RobotDealer.h"

std::random_device rd;
std::mt19937 rng(rd());

void feedbackCallback(const roboteam_msgs::RoleFeedbackConstPtr &msg) {
    auto uuid = unique_id::fromMsg(msg->token);   

    std::cout << "Received a feedback on token " << uuid << "!\n";

    if (msg->status == roboteam_msgs::RoleFeedback::STATUS_FAILURE) {
        rtt::feedbacks[uuid] = bt::Node::Status::Failure; 
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_INVALID) {
        rtt::feedbacks[uuid] = bt::Node::Status::Invalid;
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_SUCCESS) {
        rtt::feedbacks[uuid] = bt::Node::Status::Success;
    }
}

/**
 * TODO: strategy_debug_directive voor het ontvangen voor debug bomen voor robots
 */
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "StrategyNode");
    ros::NodeHandle n;
    
    std::string name = ros::this_node::getName();

    std::cout << "Name: " << name << "\n";

    ros::Rate rate(60);

    ros::Publisher directivePub = n.advertise<roboteam_msgs::RoleDirective>("role_directive", 10);
    ros::Subscriber feedbackSub = n.subscribe("role_feedback", 10, &feedbackCallback);

    std::vector<std::string> arguments(argv + 1, argv + argc);

    auto strategy = rtt::make_SimpleSideSideStrategy(n);

    // Wait for all the role nodes to come online if a param was set
    if (ros::param::has("num_role_nodes")) {
        int numNodes;
        ros::param::get("num_role_nodes", numNodes);
        std::cout << "Waiting for " << std::to_string(numNodes) << " role nodes...\n";
        while ((int) directivePub.getNumSubscribers() < numNodes) {}
    }

    rtt::RobotDealer::initialize_robots({0, 1, 2, 3, 4, 5});
    rtt::RobotDealer::initialize_role_nodes();

    while (ros::ok()) {
        ros::spinOnce();
        
        bt::Node::Status status = strategy.Update();

        if (status != bt::Node::Status::Running) {
            std::cout << "Strategy result: " << bt::statusToString(status) << ". Starting again\n";
        }
            
        rate.sleep();
    }

    return 0;
}

