#include <iostream>
#include <set>
#include <string>
#include <memory>
#include <chrono>

#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "uuid_msgs/UniqueID.h"

#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/RoleFeedback.h"
#include "roboteam_msgs/BtDebugInfo.h"
#include "roboteam_msgs/BtStatus.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RefereeCommand.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/constants.h"

#include "roboteam_tactics/treegen/NodeFactory.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/BtDebug.h"
#include "roboteam_tactics/utils/CrashHandler.h"

namespace {

    std::string const RED_BOLD_COLOR = "\e[1;31m";
    std::string const END_COLOR = "\e[0m";

} // anonymous namespace

#define RTT_CURRENT_DEBUG_TAG RoleNode

ros::Publisher pub;
ros::Publisher feedbackPub;
bt::Node::Ptr currentTree;
std::string currentTreeName;

uuid_msgs::UniqueID currentToken;
int ROBOT_ID;
std::string ROS_LOG_NAME = "RoleNode";


bool ignoring_strategy_instructions = false;

void msgCallbackRef(const roboteam_msgs::RefereeDataConstPtr& refdata) {
    rtt::LastRef::set(*refdata);
    //ROS_INFO("set ref, timestamp: %d",refdata->packet_timestamp);
}

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

    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Directive received : " << msg->tree.c_str());

    ros::NodeHandle n;

    if (msg->tree == roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE) {
        reset_tree();

        ROS_WARN_NAMED(ROS_LOG_NAME, "Received stop command");

        // Stop the robot in its tracks
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

    if (!bb->HasInt("ROBOT_ID")) {
        ROS_ERROR_NAMED(ROS_LOG_NAME, "RoleDirective received without ROBOT_ID");
    }

    if (!bb->HasInt("KEEPER_ID")) {
        ROS_ERROR_NAMED(ROS_LOG_NAME, "RoleDirective received without KEEPER_ID");
    }

    {   // Emiel: Not sure why this scope is here
//        ros::NodeHandle n;    // Emiel: not sure why this is here
        currentTreeName = msg->tree;
        currentTree = rtt::generate_rtt_node<>(msg->tree, "", bb);

        if (!currentTree)  {
            ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "Tree name is neither tree nor skill: " << msg->tree.c_str());
            return;
        }
    }
    currentToken = msg->token;

    RTT_SEND_RQT_BT_TRACE(ROBOT_ID, msg->tree, roboteam_msgs::BtDebugInfo::TYPE_ROLE, roboteam_msgs::BtStatus::STARTUP, bb->toMsg());
}

int main(int argc, char *argv[]) {
	rtt::crash::registerCrashHandlers();
    ros::init(argc, argv, "RoleNode", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "New RoleNode instance, name=" << ros::this_node::getName());



    // ====== ROBOT_ID ====== //
    std::string name = ros::this_node::getName();                               // Get name of Node
    std::string robotIDStr = name.substr(name.find_last_of("/\\") + 1 + 5);     // Chop off the leading "/robot"
    try {
        ROBOT_ID = std::stoi(robotIDStr);                                       // Convert to int
        std::string our_team = "no_color";
        ros::param::get("our_color", our_team);
        ROS_LOG_NAME = ROS_LOG_NAME + "." + our_team + "." + robotIDStr;                // Set logging name from RoleNode to RoleNode.our_color.ROBOT_ID
    } catch (...) {
        ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "Could not parse Robot ID from node name " << name.c_str() << "! Aborting");
        return 1;
    }
    // ====================== //

    // Set Hz of RoleNode
    int iterationsPerSecond = 60;
    rtt::get_PARAM_ITERATIONS_PER_SECOND(iterationsPerSecond);
    ros::Rate sleeprate(iterationsPerSecond);
    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "New RoleNode #" << ROBOT_ID << " at " << iterationsPerSecond << "Hz");



    // ====== Setup subscribers and advertisers ====== //
    // Advertise RobotCommands
    pub = n.advertise<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 100);

    // Create global robot command publisher
    rtt::GlobalPublisher<roboteam_msgs::RobotCommand> globalRobotCommandPublisher(rtt::TOPIC_COMMANDS);
    CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER;

    // Create world, geom, and ref callbacks
    rtt::WorldAndGeomCallbackCreator cb;
    rtt::LastWorld::wait_for_first_messages();
    // Subscribe to vision
    ros::Subscriber ref_sub = n.subscribe<roboteam_msgs::RefereeData> ("vision_refbox", 1000, msgCallbackRef);

    // Subscribe to RoleDirective, for receiving trees
    ros::Subscriber roleDirectiveSub = n.subscribe<roboteam_msgs::RoleDirective>(
        rtt::TOPIC_ROLE_DIRECTIVE,
        1000,
        &roleDirectiveCallback
    );
    // Advertise to RoleFeedback
    feedbackPub = n.advertise<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 10);
    // =============================================== //



    using namespace std::chrono;
    steady_clock::time_point start = steady_clock::now();
    int iters = 0;

    while (ros::ok()) {

        // Process all received messages
        ros::spinOnce();

        sleeprate.sleep();

        // If the RoleNode currently has no tree, continue
        if (!currentTree){
            continue;
        }

        // Update the tree, get its status
        bt::Node::Status status = currentTree->Update();

        // If the tree is not running anymore
        if (status == bt::Node::Status::Success
         || status == bt::Node::Status::Failure
         || status == bt::Node::Status::Invalid) {


             // Create a feedback message
            roboteam_msgs::RoleFeedback feedback;
            feedback.token = currentToken;

            // Create a blackboard
            roboteam_msgs::Blackboard bb;

            // TODO: Maybe implement bt rqt feedback here as well?

            std::string statusString = "";
            if (status == bt::Node::Status::Success) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_SUCCESS;
                feedbackPub.publish(feedback);
                statusString = "Success";
            } else
            if (status == bt::Node::Status::Invalid) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_INVALID;
                feedbackPub.publish(feedback);
                statusString = "Invalid";
            } else
            if (status == bt::Node::Status::Failure) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_FAILURE;
                feedbackPub.publish(feedback);
                statusString = "Failure";
            }

            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Tree finished. "
                    << "status=" << statusString
                    << ", tree=" << currentTreeName.c_str());
//                    << ", token=" << currentToken);

            // Reset tree
            currentTree = nullptr;

            // Stop the robot in its tracks
            pub.publish(rtt::stop_command(ROBOT_ID));
        }

        iters++;

        if ((steady_clock::now() - start) > milliseconds(5000)) {
            start = steady_clock::now();

            // ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Actual Hz = " << (iters / 5.0));
            // If the actual roleHz is lower than 80%, throw a warning
            if((iters/5.0) < iterationsPerSecond * 0.8){
                ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "RoleHz is low! " << round(iters / 5.0) << "/" << iterationsPerSecond);
            }

            iters = 0;
        }
    }

    return 0;
}
