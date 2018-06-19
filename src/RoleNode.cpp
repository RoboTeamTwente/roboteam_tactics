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

ros::Publisher pub;
ros::Publisher feedbackPub;
bt::Node::Ptr currentTree;
std::string currentTreeName = "No tree";

int ROBOT_ID;
uuid_msgs::UniqueID currentToken;
int nRolesReceived = 0;

std::string ROS_LOG_NAME = "RoleNode";

// Stores the latest refdata
void msgCallbackRef(const roboteam_msgs::RefereeDataConstPtr& refdata) {
    rtt::LastRef::set(*refdata);
}

void reset_tree() {
    currentToken = uuid_msgs::UniqueID();
    currentTree = nullptr;
    currentTreeName = "No tree";
}

/**
 * Handles Role Directive messages. Loads ROBOT_ID and KEEPER_ID, blackboard, and validates tree. Ignores messages for other robots
 * @param msg The RoleDirective received
 */
void roleDirectiveCallback(const roboteam_msgs::RoleDirectiveConstPtr &msg) {

    // Ignore any message that is not meant for this robot
    if (msg->robot_id != ROBOT_ID)
        return;

    // If this robot received the stop command, reset the tree and stop the robot
    if (msg->tree == roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE) {
        // Reset the tree
        reset_tree();
        // Stop the robot in its tracks
        pub.publish(rtt::stop_command(ROBOT_ID));
        ROS_WARN_NAMED(ROS_LOG_NAME, "Received stop command");
        return;
    }

    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Directive received : " << msg->tree.c_str());
    nRolesReceived++;

    // Get the blackboard from the role directive
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->fromMsg(msg->blackboard);

    // Check for robot ID
    if (!bb->HasInt("ROBOT_ID")) {
        ROS_ERROR_NAMED(ROS_LOG_NAME, "RoleDirective received without ROBOT_ID");
    }
    // Check for keeper ID
    if (!bb->HasInt("KEEPER_ID")) {
        ROS_ERROR_NAMED(ROS_LOG_NAME, "RoleDirective received without KEEPER_ID");
    }

    currentTreeName = msg->tree;
    currentTree = rtt::generate_rtt_node<>(msg->tree, "", bb);

    if (!currentTree)  {
        ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "Tree name is neither tree nor skill: " << msg->tree.c_str());
        return;
    }

    currentToken = msg->token;

//    RTT_SEND_RQT_BT_TRACE(ROBOT_ID, msg->tree, roboteam_msgs::BtDebugInfo::TYPE_ROLE, roboteam_msgs::BtStatus::STARTUP, bb->toMsg());
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


    // ================ MAIN LOOP ================ //
    while (ros::ok()) {

        // Process all received messages
        ros::spinOnce();

        sleeprate.sleep();

        // If the RoleNode currently has no tree, continue
        if (!currentTree){
            continue;
        }

        // If the RoleNode has not yet received a world, continue
        if(!rtt::LastWorld::have_received_first_world()){
            continue;
        }

        // Check if the RoleNode exists in the world. If not, continue
        const roboteam_msgs::World& world = rtt::LastWorld::get();
        if(!rtt::getWorldBot(ROBOT_ID, true, world)){
            // If the robot doesn't exist in the world, but does have a tree, something is wrong
            ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(1, ROS_LOG_NAME, "Robot " << ROBOT_ID << " has a tree, but doesn't exist in the world! Tree=" << currentTreeName);
            continue;
        }

        // Run the strategy tree
        bt::Node::Status status = bt::Node::Status::Invalid;
        try{
            status = currentTree->Update();
        }catch(const std::out_of_range& e){
            ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "Exception caught!");
            ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, e.what());
        }catch (const std::exception& e) {
            ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "Exception caught!");
            ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, e.what());
        } catch (const std::string& e) {
            ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "Exception caught!");
            ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, e.c_str());
        } catch (...) {
            ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "General exception caught!");
        }


        // If the tree is not running anymore
        if (status == bt::Node::Status::Success
         || status == bt::Node::Status::Failure
         || status == bt::Node::Status::Invalid) {


             // Create a feedback message
            roboteam_msgs::RoleFeedback feedback;
            feedback.token = currentToken;

            // Create a blackboard
            roboteam_msgs::Blackboard bb;

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

            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Tree finished. " << "status=" << statusString << ", tree=" << currentTreeName.c_str());

            // Reset tree
            reset_tree();

            // Stop the robot in its tracks
            pub.publish(rtt::stop_command(ROBOT_ID));
        }

        iters++;

        if ((steady_clock::now() - start) > milliseconds(5000)) {
            start = steady_clock::now();

            // ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Actual Hz = " << (iters / 5.0));
            // If the actual roleHz is lower than 80%, throw a warning
            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Role changes per second = " << round(nRolesReceived / 5.0));

            if((iters/5.0) < iterationsPerSecond * 0.8){
                ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "RoleHz is low! " << round(iters / 5.0) << "/" << iterationsPerSecond);
            }

            iters = 0;
        }
    }

    return 0;
}
