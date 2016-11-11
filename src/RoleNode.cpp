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
#include "roboteam_tactics/generated/alltrees_set.h"
#include "roboteam_tactics/generated/allskills_set.h"
#include "roboteam_tactics/generated/allskills_factory.h"

ros::Publisher roleNodeDiscoveryPublisher;
ros::Publisher feedbackPub;
bt::Node::Ptr currentTree;

bool sendNextSuccess = false;
uuid_msgs::UniqueID currentToken;
int currentRobotID = -1;

void reset_tree() {
    sendNextSuccess = false;
    currentToken = uuid_msgs::UniqueID();
    currentTree = nullptr;
    currentRobotID = -1;
}

void roleDirectiveCallback(const roboteam_msgs::RoleDirectiveConstPtr &msg) {
    std::string name = ros::this_node::getName();
    
    // Some control statements to regulate starting and stopping of rolenodes
    if (msg->node_id.empty()) {
        // Directive is meant for all
        bt::Blackboard bb;
        bb.fromMsg(msg->blackboard);
        // If robot ID is set to ours...
        if (bb.HasInt("ROBOT_ID") && bb.GetInt("ROBOT_ID") == currentRobotID) {
            // And the tree directive is empty
            if (msg->tree.empty()) {
                // Stop executing the tree and notify the strategy node
                // That we failed
                roboteam_msgs::RoleFeedback feedback;
                feedback.token = currentToken;
                feedback.status = roboteam_msgs::RoleFeedback::STATUS_FAILURE;
                feedbackPub.publish(feedback);

                reset_tree();
                return;
            }
        } else if (!bb.HasInt("ROBOT_ID")) { // No Robot ID was set...
            // And the tree is empty...
            if (msg->tree.empty()) {
                // And if we are currently controlling a robot...
                if (currentRobotID == -1) { 
                    // Stop executing it!
                    roboteam_msgs::RoleFeedback feedback;
                    feedback.token = currentToken;
                    feedback.status = roboteam_msgs::RoleFeedback::STATUS_FAILURE;
                    feedbackPub.publish(feedback);

                    reset_tree();
                    return;
                }
            }
        }

        return;
    } else if (name != msg->node_id) {
        return;
    }

    ros::NodeHandle n;

    bt::Blackboard::Ptr bb;

    // TODO: Migrate this to Dennis' Node factory
    if (rtt::alltrees_set.find(msg->tree) != rtt::alltrees_set.end()) {
        std::shared_ptr<bt::BehaviorTree> tree = std::make_shared<bt::BehaviorTree>();
        *tree = rtt::make_tree(msg->tree, n);

        bb = tree->GetBlackboard();
        bb->fromMsg(msg->blackboard);

        currentTree = tree;
    } else if (rtt::allskills_set.find(msg->tree) != rtt::allskills_set.end()) {
        bb = std::make_shared<bt::Blackboard>();
        bb->fromMsg(msg->blackboard);
        currentTree = rtt::make_skill(n, msg->tree, "", bb);
    } else {
        std::cout << "Tree name is neither tree nor skill: \"" << msg->tree << "\"\n";
        return;
    }

    currentToken = msg->token;
    if (bb->HasInt("ROBOT_ID")) {
        currentRobotID = bb->GetInt("ROBOT_ID");
    }

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

    feedbackPub = n.advertise<roboteam_msgs::RoleFeedback>("role_feedback", 10);

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
            std::cout << "Finished a RoleDirective. Sending feedback\n";

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

            currentTree = nullptr;
        }

        fps60.sleep();
    }

    return 0;
}
