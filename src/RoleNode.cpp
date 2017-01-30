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
#include "roboteam_msgs/BtDebugInfo.h"
#include "roboteam_msgs/BtStatus.h"
#include "roboteam_msgs/RobotCommand.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/constants.h"

#include "roboteam_tactics/treegen/NodeFactory.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/BtDebug.h"


#define RTT_CURRENT_DEBUG_TAG RoleNode

ros::Publisher pub;
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
    // std::cout << ROBOT_ID << ": Got message!\n";

    std::string name = ros::this_node::getName();

    // std::cout << "Message: " << *msg << "\n";
    
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

    // std::cout << ROBOT_ID  << ": Got tree: " << msg->tree << "\n";

    if (msg->tree == roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE) {
        reset_tree();

        // Stop the robot in its tracks
        pub.publish(rtt::stop_command(ROBOT_ID));

        // std::cout << ROBOT_ID << ": Stop executing tree!\n";

        return;
    } else if (msg->tree == roboteam_msgs::RoleDirective::IGNORE_STRATEGY_INSTRUCTIONS) {
        reset_tree();
        ignoring_strategy_instructions = true;

        // std::cout << "Ignoring strategy instructions!\n";

        return;
    } else if (msg->tree == roboteam_msgs::RoleDirective::STOP_IGNORING_STRATEGY_INSTRUCTIONS) {
        ignoring_strategy_instructions = false;
        // std::cout << "Ignoring strategy instructions no!\n";
        return;
    }

    // std::cout << "Ignoring strategy? ";

    if (ignoring_strategy_instructions) {
        // std::cout << "Yes!\n";
        return;
    }

    // std::cout << "No!\n";

    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->fromMsg(msg->blackboard);

    try {
        ros::NodeHandle n;
        currentTreeName = msg->tree;
        currentTree = rtt::generate_rtt_node<>(msg->tree, "", bb);
    } catch (...) {
        ROS_ERROR("Tree name is neither tree nor skill: \"%s\"",  msg->tree.c_str());
        return;
    }

    currentToken = msg->token;

    RTT_DEBUG("Robot %i starts executing tree: %s.\n", ROBOT_ID, msg->tree.c_str());

    RTT_SEND_RQT_BT_TRACE(msg->tree, roboteam_msgs::BtDebugInfo::TYPE_ROLE, roboteam_msgs::BtStatus::STARTUP, bb->toMsg());
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "RoleNode", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    pub = n.advertise<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 100);

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
    rtt::get_PARAM_ITERATIONS_PER_SECOND(iterationsPerSecond);
    ros::Rate sleeprate(iterationsPerSecond);
    RTT_DEBUG("Iterations per second: %i\n", iterationsPerSecond);

    // Create global robot command publisher
    rtt::GlobalPublisher<roboteam_msgs::RobotCommand> globalRobotCommandPublisher(rtt::TOPIC_COMMANDS);
    CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER;
    
    // Create world & geom callbacks
    rtt::WorldAndGeomCallbackCreator cb;
    rtt::LastWorld::wait_for_first_messages();

    // For receiving trees
    ros::Subscriber roleDirectiveSub = n.subscribe<roboteam_msgs::RoleDirective>(
        rtt::TOPIC_ROLE_DIRECTIVE,
        1000,
        &roleDirectiveCallback
        );

    feedbackPub = n.advertise<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 10);

    while (ros::ok()) {
        ros::spinOnce();
        
        sleeprate.sleep();

        if (!currentTree){
            continue;
        }

        // if (ROBOT_ID == 1) RTT_DEBUGLN("Updating");
        bt::Node::Status status = currentTree->Update();
        // if (ROBOT_ID == 1) RTT_DEBUGLN("Done updating");

        if (status == bt::Node::Status::Success
                 || status == bt::Node::Status::Failure
                 || status == bt::Node::Status::Invalid) {
            RTT_DEBUGLN("Robot %i has finished tree %s", ROBOT_ID, currentTreeName.c_str());


            roboteam_msgs::RoleFeedback feedback;
            feedback.token = currentToken;

            roboteam_msgs::Blackboard bb;

            // TODO: Maybe implement bt rqt feedback here as well?

            if (status == bt::Node::Status::Success) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_SUCCESS;
                feedbackPub.publish(feedback);
            } else if (status == bt::Node::Status::Invalid) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_INVALID;
                feedbackPub.publish(feedback);
            } else if (status == bt::Node::Status::Failure) {
                RTT_DEBUGLN("Role node failed! ID: %d", ROBOT_ID);
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_FAILURE ;
                feedbackPub.publish(feedback);
            }

            currentTree = nullptr;
            
            // Stop the robot in its tracks
            pub.publish(rtt::stop_command(ROBOT_ID));
        }
    }

    return 0;
}
