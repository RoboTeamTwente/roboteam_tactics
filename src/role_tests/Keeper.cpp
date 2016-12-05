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

void worldStateCallback(const roboteam_msgs::WorldConstPtr& world, bt::BehaviorTree* tree2, bt::Blackboard::Ptr bb2) {
    rtt::LastWorld::set(*world);

    roboteam_msgs::World getworld = rtt::LastWorld::get();
    roboteam_msgs::WorldBall ball = getworld.ball;
    roboteam_utils::Vector2 center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
    bb2->SetDouble("RotateAroundPoint_A_centerx", center.x);
    bb2->SetDouble("RotateAroundPoint_A_centery", center.y);

    bt::Node::Status status = tree2->Update();
    if (status == bt::Node::Status::Success) {
        success = true;
    }
    if (status == bt::Node::Status::Failure) {
        failure = true;
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "KeeperTactic");
    ros::NodeHandle n;

    auto role2 = make_CoolTree();
    auto bb2 = role2.GetBlackboard();

    bb2->SetInt("ROBOT_ID", 2);
    bb2->SetBool("GetBall_A_intercept", true);
    bb2->SetDouble("GetBall_A_getBallAtX", 2.9);
    bb2->SetDouble("GetBall_A_getBallAtY", 0.0);
    bb2->SetDouble("GetBall_A_getBallAtTime", 5.0);
    bb2->SetString("RotateAroundPoint_A_center", "ball");
    bb2->SetDouble("RotateAroundPoint_A_faceTowardsPosx", -3.0);
    bb2->SetDouble("RotateAroundPoint_A_faceTowardsPosy", 0.0);
    bb2->SetDouble("RotateAroundPoint_A_w",3.0);
    bb2->SetDouble("RotateAroundPoint_A_radius", 0.09);

    ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&worldStateCallback, _1, &role2, bb2));

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
