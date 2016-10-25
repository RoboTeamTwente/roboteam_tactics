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

bool success1;
bool success;
bool failure;

void worldStateCallback(const roboteam_msgs::WorldConstPtr& world, bt::BehaviorTree* tree, bt::Blackboard::Ptr bb, bt::BehaviorTree* tree2, bt::Blackboard::Ptr bb2) {
    rtt::LastWorld::set(*world);

    roboteam_msgs::World getworld = rtt::LastWorld::get();
    roboteam_msgs::WorldBall ball = getworld.ball;
    roboteam_utils::Vector2 center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
    bb->SetDouble("RotateAroundPoint_A_centerx", center.x);
    bb->SetDouble("RotateAroundPoint_A_centery", center.y);
    bb2->SetDouble("RotateAroundPoint_A_centerx", center.x);
    bb2->SetDouble("RotateAroundPoint_A_centery", center.y);

    if (success1 != true) {
        bt::Node::Status status = tree->Update();
        if (status == bt::Node::Status::Success) {
            success1 = true;
        }
    }

    bt::Node::Status status = tree2->Update();
    if (status == bt::Node::Status::Success) {
        success = true;
    }
    if (status == bt::Node::Status::Failure) {
        failure = true;
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "tactics");
    ros::NodeHandle n;

    roboteam_utils::Vector2 passTo = roboteam_utils::Vector2(1.0, -1.0);

    auto role = make_CoolTree(n);
    auto bb = role.GetBlackboard();

    auto role2 = make_CoolTree(n);
    auto bb2 = role2.GetBlackboard();

    bb->SetInt("ROBOT_ID", 1);
    bb->SetString("RotateAroundPoint_A_center", "ball");
    bb->SetBool("GetBall_A_intercept", false);
    bb->SetDouble("RotateAroundPoint_A_faceTowardsPosx", passTo.x);
    bb->SetDouble("RotateAroundPoint_A_faceTowardsPosy", passTo.y);
    bb->SetDouble("RotateAroundPoint_A_w",1.0);
    bb->SetDouble("RotateAroundPoint_A_radius", 0.09);

    bb2->SetInt("ROBOT_ID", 2);
    bb2->SetBool("GetBall_A_intercept", true);
    bb2->SetDouble("GetBall_A_getBallAtX", passTo.x);
    bb2->SetDouble("GetBall_A_getBallAtY", passTo.y);
    bb2->SetDouble("GetBall_A_getBallAtTime", 5.0);
    bb2->SetString("RotateAroundPoint_A_center", "ball");
    bb2->SetDouble("RotateAroundPoint_A_faceTowardsPosx", 3.0);
    bb2->SetDouble("RotateAroundPoint_A_faceTowardsPosy", 0.0);
    bb2->SetDouble("RotateAroundPoint_A_w",1.0);
    bb2->SetDouble("RotateAroundPoint_A_radius", 0.09);

    ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&worldStateCallback, _1, &role, bb, &role2, bb2));

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
