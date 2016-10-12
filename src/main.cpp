#include <array>
#include <iostream>
#include <unordered_map>
#include <memory>

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_tactics/alltrees.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

void world_state_callback(const roboteam_msgs::World::ConstPtr& msg) {
    LastWorld::set(*msg);
    std::cout << "Received a world.\n";
}

bt::BehaviorTree strategy = make_BasicStrategy();

void run_ai_cycle() {
    // If there is no tactic, pick one
    if (!pickedTactic) {
        strategy.Update();

        // Let tactic distribute roles
        pickedTactic.distribute_roles();
    } else if (false) { // Check somehow if the tactic is still applicable
        // If not, let strategy pick a new one
        strategy.Update();
        
        // Let tactic distribute roles
        pickedTactic.distribute_roles();
    }

    // Let the roles update s.t. the robots start moving
    pickedTactic.update_roles();
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
    
    return 0;
}

} // rtt

int main(int argc, char **argv) {
    return rtt::main(argc, argv);
}
