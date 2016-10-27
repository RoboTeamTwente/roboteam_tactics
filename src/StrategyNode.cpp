#include <algorithm>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "std_msgs/Empty.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/generated/alltrees_list.h"

/**
 * Gets all the nodes subscribed to topic topic, relative
 * to the current node.
 */
std::vector<std::string> getNodesSubscribedTo(std::string topic)
{
    // Adapted from ros::master::getNodes()
    // Do the xmlrpc request to master
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getSystemState", args, result, payload, true)) {
        // Return an empty list if something went wrong
        return {};
    }

    // See: http://wiki.ros.org/ROS/Master_API
    // and then getSystemState for the format
    std::vector<std::string> nodes;
    XmlRpc::XmlRpcValue subscribers = payload[1];
    for (int i = 0; i < subscribers.size(); ++i) {
        XmlRpc::XmlRpcValue topicInfo = subscribers[i];
        XmlRpc::XmlRpcValue topicName = topicInfo[0];
        XmlRpc::XmlRpcValue topicSubscribers = topicInfo[1];

        if (topicName == topic) {
            for (int j = 0; j < topicSubscribers.size(); ++j) {
                nodes.push_back(topicSubscribers[j]);
            }
            break;
        }
    }

    return nodes;
}

std::random_device rd;
std::mt19937 rng(rd());

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "StrategyNode");
    ros::NodeHandle n;
    
    std::string name = ros::this_node::getName();

    std::cout << "Name: " << name << "\n";

    ros::Rate rate(60);

    ros::Publisher directivePub = n.advertise<roboteam_msgs::RoleDirective>("role_directive", 10);

    int cycles = 0;

    std::vector<std::string> arguments(argv + 1, argv + argc);

    while (ros::ok()) {
        ros::spinOnce();

        if (cycles >= 60) {
            cycles = 0;
            
            auto workerNodes = getNodesSubscribedTo("/role_directive");
            std::cout << "Worker size: " << workerNodes.size() << "\n";

            if (workerNodes.size() >= 0) {
                std::uniform_int_distribution<uint32_t> uint_dist(0, workerNodes.size() - 1);
                std::string recipient = workerNodes.at(uint_dist(rng));

                uint_dist = std::uniform_int_distribution<uint32_t>(0, rtt::alltrees_list.size() - 1);
                std::string tree = rtt::alltrees_list.at(uint_dist(rng));

                std::cout << "Making node " << recipient << " do " << tree << "\n";

                roboteam_msgs::RoleDirective directive;
                directive.node_id = recipient;
                directive.tree = tree;

                directivePub.publish(directive);
            }
        }

        rate.sleep();
        cycles++;
    }

    return 0;
}

