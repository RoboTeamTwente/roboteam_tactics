#include <iostream>
#include <string>

#include "ros/ros.h"

#include "std_msgs/Empty.h"

std::set<std::string> known_nodes;

void callback(const ros::MessageEvent<std_msgs::Empty const>& event) {
    const std::string& publisher_name = event.getPublisherName();

    std::cout << "Heard: " << publisher_name << "\n";

    if (known_nodes.find(publisher_name) == known_nodes.end()) {
        known_nodes.insert(publisher_name);
        std::cout << "Got to know " << publisher_name << "!\n";
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "StrategyNode");
    ros::NodeHandle n;
    
    std::string name = ros::this_node::getName();

    std::cout << "Name: " << name << "\n";

    ros::Rate rate(60);

    ros::Subscriber sub = n.subscribe("role_node_discovery", 10, callback);

    while (ros::ok()) {
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}

