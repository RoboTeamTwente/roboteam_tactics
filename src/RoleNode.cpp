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
#include "roboteam_tactics/utils/EmptyNode.h"
#include "roboteam_tactics/generated/alltrees_factory.h"
#include "roboteam_tactics/generated/alltrees_set.h"
#include "roboteam_tactics/generated/allskills_set.h"
#include "roboteam_tactics/generated/allskills_factory.h"

ros::Publisher roleNodeDiscoveryPublisher;
bt::Node::Ptr currentTree;

bool sendNextSuccess = false;
uuid_msgs::UniqueID currentToken;

void roleDirectiveCallback(const roboteam_msgs::RoleDirectiveConstPtr &msg) {
    std::string name = ros::this_node::getName();
    
    if (name != msg->node_id) {
        return;
    }

    ros::NodeHandle n;

    bt::Blackboard::Ptr bb;

    if (rtt::alltrees_set.find(msg->tree) != rtt::alltrees_set.end()) {
        std::shared_ptr<bt::BehaviorTree> tree = std::make_shared<bt::BehaviorTree>();
        *tree = rtt::make_tree(msg->tree, n);

        bb = tree->GetBlackboard();
        bb->fromMsg(msg->blackboard);

        currentTree = tree;
    } else if (rtt::allskills_set.find(msg->tree) != rtt::allskills_set.end()) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        bb->fromMsg(msg->blackboard);
        currentTree = rtt::make_skill(n, msg->tree, "", bb);
    } else {
        std::cout << "Tree name is neither tree nor skill: \"" << msg->tree << "\"\n";
    }

    currentToken = msg->token;

    sendNextSuccess = true;

    std::cout << "It's for me, " << name << ". I have to start executing tree " << msg->tree << ". My robot is: " << bb->GetInt("ROBOT_ID") << "\n";
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

    // For receiving trees
    ros::Subscriber roleDirectiveSub = n.subscribe<roboteam_msgs::RoleDirective>(
        "role_directive",
        1000,
        &roleDirectiveCallback
        );

    ros::Publisher feedbackPub = n.advertise<roboteam_msgs::RoleFeedback>("role_feedback", 10);

    while (ros::ok()) {
        ros::spinOnce();

        if (!currentTree){
            fps60.sleep();
            continue;
        }

        bt::Node::Status status = currentTree->Update();

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
