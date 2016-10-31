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
#include "roboteam_utils/Vector2.h"

namespace rtt {

bool success;
bool failure;

void worldStateCallback(const roboteam_msgs::WorldConstPtr& world, bt::BehaviorTree* tree, bt::Blackboard::Ptr bb) {
    rtt::LastWorld::set(*world);

    roboteam_msgs::World getworld = rtt::LastWorld::get();
    roboteam_msgs::WorldBall ball = getworld.ball;
    roboteam_utils::Vector2 center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);

   
    bt::Node::Status status = tree->Update();
    if (status == bt::Node::Status::Success) {
        success = true;
    }
    if (status == bt::Node::Status::Failure) {
        failure = true;
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "KickAtGoalTactic");
    ros::NodeHandle n;

    roboteam_utils::Vector2 passTo = roboteam_utils::Vector2(3.0, 0.1);

    auto role = make_CoolTree(n);
    auto bb = role.GetBlackboard();

    bb->SetInt("ROBOT_ID", 0);
    bb->SetBool("GetBall_A_intercept", false);
    bb->SetDouble("RotateAroundPoint_A_faceTowardsPosx", passTo.x);
    bb->SetDouble("RotateAroundPoint_A_faceTowardsPosy", passTo.y);
    
    // params for aimAt are set in tree

    ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&worldStateCallback, _1, &role, bb));

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
