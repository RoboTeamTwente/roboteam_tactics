#include <algorithm>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "std_msgs/Empty.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/generated/alltrees.h"
#include "roboteam_tactics/generated/alltrees_list.h"

std::random_device rd;
std::mt19937 rng(rd());

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

    int cycles = 0;

    std::vector<std::string> arguments(argv + 1, argv + argc);

    auto strategy = rtt::make_SideSideStrategy(n);

    while (ros::ok()) {
        ros::spinOnce();

        // if (cycles >= 60) {
            // cycles = 0;
            
            // auto workerNodes = getNodesSubscribedTo("/role_directive");
            // std::cout << "Worker size: " << workerNodes.size() << "\n";

            // if (workerNodes.size() >= 0) {
                // std::uniform_int_distribution<uint32_t> uint_dist(0, workerNodes.size() - 1);
                // std::string recipient = workerNodes.at(uint_dist(rng));

                // uint_dist = std::uniform_int_distribution<uint32_t>(0, rtt::alltrees_list.size() - 1);
                // std::string tree = rtt::alltrees_list.at(uint_dist(rng));

                // std::cout << "Making node " << recipient << " do " << tree << "\n";

                // roboteam_msgs::RoleDirective directive;
                // directive.node_id = recipient;
                // directive.tree = tree;

                // directivePub.publish(directive);
            // }
        // }
        
        strategy.Update();

        rate.sleep();
        cycles++;
    }

    return 0;
}

