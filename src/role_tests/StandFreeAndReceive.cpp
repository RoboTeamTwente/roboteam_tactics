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

    roboteam_msgs::World getworld = rtt::LastWorld::get();
    roboteam_msgs::WorldBall ball = getworld.ball;
    roboteam_utils::Vector2 center = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
    int myID = 1;
    roboteam_utils::Vector2 myPos(getworld.us.at(myID).pos.x, getworld.us.at(myID).pos.y);
    bb->SetDouble("GetBall_A_getBallAtX", myPos.x);
    bb->SetDouble("GetBall_A_getBallAtY", myPos.y);
   
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
    
    ros::init(argc, argv, "StandFreeAndReceive");
    ros::NodeHandle n;
    n.setParam("/testParam", 1);

    auto role = make_SuperCoolTree(n);
    auto bb = role.GetBlackboard();

    bb->SetInt("ROBOT_ID", 1);
    bb->SetInt("StandFree_A_theirID", 0);
    bb->SetBool("StandFree_A_setRosParam", true);
    bb->SetString("StandFree_A_whichTeam", "us");
    bb->SetDouble("StandFree_A_distanceFromPoint", 0.4);
    bb->SetBool("GetBall_A_setRosParam", false);
    bb->SetBool("GetBall_A_intercept", true);
    bb->SetDouble("GetBall_A_getBallAtX", 0.0);
    bb->SetDouble("GetBall_A_getBallAtY", 0.0);
    bb->SetString("AimAt_A_At", "theirgoal");
    
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
