#include <iostream>
#include <set>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "uuid_msgs/UniqueID.h"

#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/RoleFeedback.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/generated/alltrees_factory.h"

ros::Publisher roleNodeDiscoveryPublisher;
bt::BehaviorTree currentTree;
bt::BehaviorTree emptyTree;

bool sendNextSuccess = false;
uuid_msgs::UniqueID currentToken;

void roleDirectiveCallback(const roboteam_msgs::RoleDirectiveConstPtr &msg) {
    std::string name = ros::this_node::getName();
    
    if (name != msg->node_id) {
        return;
    }

    std::cout << "It's for me, " << name << ". I have to start executing tree " << msg->tree << "\n";

    ros::NodeHandle n;

    currentTree = rtt::make_tree(msg->tree, n);

    auto bb = currentTree.GetBlackboard();
    
    bb->fromMsg(msg->blackboard);

    std::cout << "My robot: " << std::to_string(bb->GetInt("ROBOT_ID")) << "\n";

    currentToken = msg->token;

    sendNextSuccess = true;
}

void worldStateCallback(const roboteam_msgs::WorldConstPtr& world) {
    rtt::LastWorld::set(*world);
}

void fieldUpdateCallback(const roboteam_msgs::GeometryDataConstPtr& geom) {
    rtt::LastWorld::set_field(geom->field);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "RoleNode", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::string name = ros::this_node::getName();

    ros::Rate fps60(60);

    ros::Subscriber subWorld = n.subscribe<roboteam_msgs::World> ("world_state", 10, &worldStateCallback);
    ros::Subscriber subField = n.subscribe("vision_geometry", 10, &fieldUpdateCallback);
    
    auto fakeSequence = std::make_shared<bt::Sequence>();
    emptyTree.SetRoot(fakeSequence);
    currentTree = emptyTree;

    // For receiving trees
    ros::Subscriber roleDirectiveSub = n.subscribe<roboteam_msgs::RoleDirective>(
        "role_directive",
        1000,
        &roleDirectiveCallback
        );

    ros::Publisher feedbackPub = n.advertise<roboteam_msgs::RoleFeedback>("role_feedback", 10);

    while (ros::ok()) {
        ros::spinOnce();

        bt::Node::Status status = currentTree.Update();

        auto bb = currentTree.GetBlackboard();
        if (bb->GetInt("ROBOT_ID") == 1) {
            std::cout << "sendNextSuccess: " << std::to_string(sendNextSuccess) << "\n";
            if (status ==  bt::Node::Status::Success) {
                std::cout << "Success\n";
            } else if (status == bt::Node::Status::Failure) {
                std::cout << "Failure\n";
            } else if (status == bt::Node::Status::Invalid) {
                std::cout << "Invalid\n";
            } else {
                std::cout << "Running\n";
            }
        }

        if (sendNextSuccess && 
                (status == bt::Node::Status::Success
                 || status == bt::Node::Status::Failure
                 || status == bt::Node::Status::Invalid)) {
            std::cout << "Sending feedback!\n";

            roboteam_msgs::RoleFeedback feedback;
            feedback.token = currentToken;

            if (status == bt::Node::Status::Success) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_SUCCESS;
                feedbackPub.publish(feedback);
            } else if (status == bt::Node::Status::Invalid) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_INVALID;
                feedbackPub.publish(feedback);
            } else if (status == bt::Node::Status::Failure) {
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_FAILURE ;
                feedbackPub.publish(feedback);
            }

            sendNextSuccess = false;
        }

        fps60.sleep();
    }

    return 0;
}
