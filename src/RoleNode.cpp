#include <iostream>
#include <set>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/generated/alltrees_factory.h"

ros::Publisher roleNodeDiscoveryPublisher;

void roleDirectiveCallback(const roboteam_msgs::RoleDirectiveConstPtr &msg) {
    std::string name = ros::this_node::getName();
    
    if (name != msg->node_id) {
        return;
    }

    std::cout << "It's for me, " << name << ". I have to start executing tree " << msg->tree << "\n";
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "RoleNode", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    
    std::string name = ros::this_node::getName();

    std::cout << "Name: " << name << "\n";

    ros::Rate fps60(60);
    
    // For receiving trees
    ros::Subscriber roleDirectiveSub = n.subscribe<roboteam_msgs::RoleDirective>(
        "role_directive",
        1000,
        &roleDirectiveCallback
        );

    while (ros::ok()) {
        ros::spinOnce();

        fps60.sleep();
    }

    return 0;
}
