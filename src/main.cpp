#include <array>
#include <iostream>
#include <unordered_map>
#include <memory>

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_tactics/generated/alltrees.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/PickedTactic.h"

namespace rtt {

void world_state_callback(const roboteam_msgs::World::ConstPtr& msg) {
    LastWorld::set(*msg);
}

bt::BehaviorTree strategy;
int ai_count = 0;

void run_ai_cycle() {
    // If there is no tactic, pick one, or:
    if (!pickedTactic || (ai_count % 8 == 0)) { // Check somehow if the tactic is still applicable
        strategy.Update();

        // Let tactic distribute roles
        pickedTactic->distribute_roles();
    }

    // Let the roles update s.t. the robots start moving
    pickedTactic->update_roles();

    // Stop after 100 iterations
    if (++ai_count == 100) {
        ros::shutdown();
    }

    std::cout << "Finished an AI cycle\n";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tactics");

    ros::NodeHandle n;

    auto role = make_CoolTree(n);

    auto bb = role.GetBlackboard();
    bb->SetInt("ROBOT_ID", 0);

    bb->SetDouble("RotateAroundPoint_", 1);
    bb->SetDouble("GoToPos_first_yGoal", 0);
    bb->SetDouble("GoToPos_second_xGoal", -1);
    bb->SetDouble("GoToPos_second_yGoal", 0);

    ros::Subscriber sub = n.subscribe("world_state", 1, world_state_callback);

    while(ros::ok()) {
        ros::spinOnce();

        role.Update();
    }
    
    return 0;
}

} // rtt

int main(int argc, char **argv) {
    return rtt::main(argc, argv);
}
