#include <array>
#include <iostream>
#include <unordered_map>
#include <memory>

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"

#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_tactics/generated/alltrees.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/LastRef.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

bool success1;
bool success;
bool failure;

void worldStateCallback(const roboteam_msgs::WorldConstPtr& world, bt::BehaviorTree* tree, bt::Blackboard::Ptr bb, bt::BehaviorTree* tree2, bt::Blackboard::Ptr bb2) {
    rtt::LastWorld::set(*world);
    
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

void refStateCallback(const roboteam_msgs::RefereeData refCommand) {
    rtt::LastRef::set(refCommand);
}

void fieldUpdateCallback(const roboteam_msgs::GeometryDataConstPtr& geom) {
    rtt::LastWorld::set_field(geom->field);
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "tactics");
    ros::NodeHandle n;

    roboteam_utils::Vector2 passTo = roboteam_utils::Vector2(-1.0, 1.0);

    auto role = make_CoolTree();
    auto bb = role.GetBlackboard();

    auto role2 = make_CoolTree();
    auto bb2 = role2.GetBlackboard();

    bb->SetInt("ROBOT_ID", 1);
    bb->SetBool("GetBall_A_intercept", false);
    bb->SetString("AimAt_A_At", "robot");
    bb->SetInt("AimAt_A_AtRobot", 2);

    bb2->SetInt("ROBOT_ID", 2);
    bb2->SetBool("GetBall_A_intercept", true);
    bb2->SetDouble("GetBall_A_getBallAtX", passTo.x);
    bb2->SetDouble("GetBall_A_getBallAtY", passTo.y);
    bb2->SetDouble("GetBall_A_getBallAtTime", 5.0);
    bb2->SetString("AimAt_A_At", "theirgoal");

    ros::Subscriber subWorld = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&worldStateCallback, _1, &role, bb, &role2, bb2));
    ros::Subscriber subField = n.subscribe("vision_geometry", 10, &fieldUpdateCallback);
	ros::Subscriber subRef = n.subscribe("vision_referee", 10, &refStateCallback);


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
