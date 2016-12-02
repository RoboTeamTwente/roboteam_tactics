#include <array>
#include <iostream>
#include <unordered_map>
#include <memory>

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_msgs/GeometryData.h"
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

void fieldUpdateCallback(const roboteam_msgs::GeometryDataConstPtr& geom) {
    rtt::LastWorld::set_field(geom->field);
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "KickAtGoal");
    ros::NodeHandle n;

    // roboteam_utils::Vector2 passTo = roboteam_utils::Vector2(3.0, 0.1);

    auto role = make_CoolTree();
    auto bb = role.GetBlackboard();

    bb->SetInt("ROBOT_ID", 0);
    bb->SetBool("GetBall_A_intercept", false);
    bb->SetBool("AimAt_A_setRosParam", true);
    bb->SetString("AimAt_A_At", "robot");
    bb->SetInt("AimAt_A_AtRobot", 1);
    // bb->SetDouble("RotateAroundPoint_A_faceTowardsPosx", passTo.x);
    // bb->SetDouble("RotateAroundPoint_A_faceTowardsPosy", passTo.y);
    
    // params for aimAt are set in tree

    ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&worldStateCallback, _1, &role, bb));
    ros::Subscriber subField = n.subscribe("vision_geometry", 10, &fieldUpdateCallback);

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
