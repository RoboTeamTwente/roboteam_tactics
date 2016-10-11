#include <array>
#include <iostream>
#include <unordered_map>
#include <memory>

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Aggregator.h"

#include "roboteam_tactics/Skill.h"

namespace rtt {

void world_state_callback(const roboteam_msgs::World::ConstPtr& msg) {
    LastWorld::set(*msg);
    std::cout << "Received a world.\n";
}

void run_ai_cycle() {
    // Have a look at our trees here
}

Aggregator aggregator;

int main(int argc, char **argv) {
    ros::init(argc, argv, "tactics");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("world_state", 1, world_state_callback);

    while(ros::ok()) {
        ros::spinOnce();

        run_ai_cycle();
    }
}

}

int main(int argc, char **argv) {
    return rtt::main(argc, argv);
}
