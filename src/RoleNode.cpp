#include <iostream>
#include <set>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "RoleNode", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    
    std::string name = ros::this_node::getName();

    std::cout << "Name: " << name << "\n";

    ros::Rate fps60(60);
    
    ros::Publisher pub = n.advertise<std_msgs::Empty>("role_node_discovery", 1);    

    {
        std::cout << "Waiting for a strategy node...\n";

        while (ros::ok() && pub.getNumSubscribers() == 0) {
            ros::spinOnce();
            fps60.sleep();
        }

        std::cout << "Found one! Sending discovery message...\n";

        std_msgs::Empty empty;
        pub.publish(empty);

        std::cout << "Discovery message sent. Waint for instructions at 60 fps\n";
    }

    while (ros::ok()) {
        ros::spinOnce();

        fps60.sleep();
    }

    return 0;
}
