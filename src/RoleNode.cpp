#include <iostream>
#include <set>
#include <string>
#include <memory>

#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "uuid_msgs/UniqueID.h"

#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/RoleFeedback.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/NodeFactory.h"
#include "roboteam_tactics/generated/alltrees_factory.h"
#include "roboteam_tactics/generated/alltrees_set.h"
#include "roboteam_tactics/generated/allskills_set.h"
#include "roboteam_tactics/generated/allskills_factory.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG RoleNode

ros::Publisher feedbackPub;
bt::Node::Ptr currentTree;
std::string currentTreeName;

uuid_msgs::UniqueID currentToken;
int ROBOT_ID;
bool ignoring_strategy_instructions = false;

void reset_tree() {
    currentToken = uuid_msgs::UniqueID();
    currentTree = nullptr;
}

void roleDirectiveCallback(const roboteam_msgs::RoleDirectiveConstPtr &msg) {
    std::string name = ros::this_node::getName();
    
    // Some control statements to regulate starting and stopping of rolenodes
    if (msg->robot_id == roboteam_msgs::RoleDirective::ALL_ROBOTS) {
        // Continue
    } else if (msg->robot_id == ROBOT_ID) {
        // Also continue;
    } else {
        // Message is not for us!
        return;
    }

    ros::NodeHandle n;

    if (msg->tree == roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE) {
        reset_tree();

        // Stop the robot in its tracks
        auto& pub = rtt::get_robotcommand_publisher();
        pub.publish(rtt::stop_command(ROBOT_ID));

        return;
    } else if (msg->tree == roboteam_msgs::RoleDirective::IGNORE_STRATEGY_INSTRUCTIONS) {
        reset_tree();
        ignoring_strategy_instructions = true;
        return;
    } else if (msg->tree == roboteam_msgs::RoleDirective::STOP_IGNORING_STRATEGY_INSTRUCTIONS) {
        ignoring_strategy_instructions = false;
        return;
    }

    if (ignoring_strategy_instructions) {
        return;
    }

    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->fromMsg(msg->blackboard);

    try {
        ros::NodeHandle n;
        currentTreeName = msg->tree;
        currentTree = rtt::generate_node(n, msg->tree, "", bb);
    } catch (...) {
        ROS_ERROR("Tree name is neither tree nor skill: \"%s\"",  msg->tree.c_str());
        return;
    }

    currentToken = msg->token;

    RTT_DEBUG("Robot ID: %i. Executing tree: %s.\n", ROBOT_ID, msg->tree.c_str());
}

// void worldStateCallback(const roboteam_msgs::WorldConstPtr& world) {
    // rtt::LastWorld::set(*world);
// }

// void fieldUpdateCallback(const roboteam_msgs::GeometryDataConstPtr& geom) {
    // rtt::LastWorld::set_field(geom->field);
// }

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "RoleNode", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::string name = ros::this_node::getName();
    // Chop off the leading "/robot"
    std::string robotIDStr = name.substr(name.find_last_of("/\\") + 1 + 5);
    // Convert to int
    try {
        ROBOT_ID = std::stoi(robotIDStr);
    } catch (...) {
        ROS_ERROR("Could not parse Robot ID from node name \"%s\". Aborting.", name.c_str());
        return 1;
    }

    int iterationsPerSecond = 60;
    ros::param::get("role_iterations_per_second", iterationsPerSecond);
    ros::Rate sleeprate(iterationsPerSecond);
    RTT_DEBUG("Iterations per second: %i\n", iterationsPerSecond);
    
    rtt::LastWorld::initialise_lastworld();

    // For receiving trees
    ros::Subscriber roleDirectiveSub = n.subscribe<roboteam_msgs::RoleDirective>(
        "role_directive",
        1000,
        &roleDirectiveCallback
        );

    feedbackPub = n.advertise<roboteam_msgs::RoleFeedback>("role_feedback", 10);

    while (ros::ok()) {
        ros::spinOnce();

        if (!currentTree){
            sleeprate.sleep();
            continue;
        }

        bt::Node::Status status = currentTree->Update();

        if (status == bt::Node::Status::Success
                 || status == bt::Node::Status::Failure
                 || status == bt::Node::Status::Invalid) {
            RTT_DEBUG("Robot %i has finished tree %s\n", ROBOT_ID, currentTreeName.c_str());

            roboteam_msgs::RoleFeedback feedback;
            feedback.token = currentToken;

            if (status == bt::Node::Status::Success) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_SUCCESS;
                feedbackPub.publish(feedback);
            } else if (status == bt::Node::Status::Invalid) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_INVALID;
                feedbackPub.publish(feedback);
            } else if (status == bt::Node::Status::Failure) {
                std::cout << "Role node failed! ID: " << ROBOT_ID << "\n";
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_FAILURE ;
                feedbackPub.publish(feedback);
            }

            currentTree = nullptr;
        }

        sleeprate.sleep();
    }

    return 0;
}
