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
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

bool success;
bool failure;

void worldStateCallback(const roboteam_msgs::WorldConstPtr& world, bt::BehaviorTree* tree, bt::Blackboard::Ptr bb) {
    rtt::LastWorld::set(*world);
   
    bt::Node::Status status = tree->Update();
    if (status == bt::Node::Status::Success) {
        success = true;
    }
    if (status == bt::Node::Status::Failure) {
        failure = true;
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "doStrategy");
    ros::NodeHandle n;
	ROS_INFO("doing strategy");

    auto role = make_BasicStrategy();
    auto bb = role.GetBlackboard();

    bb->SetInt("ROBOT_ID", 0);
    
    // params for aimAt are set in tree

    ros::Subscriber sub = n.subscribe<roboteam_msgs::World> (TOPIC_WOLRD_STATE, 1000, boost::bind(&worldStateCallback, _1, &role, bb));

    while(ros::ok()) {
        ros::spinOnce();
        if (success) {
            ROS_INFO("finished");
            break;
        }
        if (failure) {
            ROS_INFO("failed :(");
            break;
        }
    }
    return 0;
}

} // rtt

int main(int argc, char **argv) {
    return rtt::main(argc, argv);
}
