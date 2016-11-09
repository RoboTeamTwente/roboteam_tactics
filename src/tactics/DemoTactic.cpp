#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>

#include "unique_id/unique_id.h"

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/DemoTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/LastWorld.h"

namespace rtt {

DemoTactic::DemoTactic(bt::Blackboard::Ptr blackboard)
        : Tactic(blackboard) 
        {}

template<typename T>
void delete_from_vector(std::vector<T> &items, const T &item) {
    auto it = std::find(items.begin(), items.end(), item);
    if (it != items.end()) {
        items.erase(it);
    }
}

void DemoTactic::Initialize() {
    tokens.clear();

    std::cout << "Initializing demo tactic";

    roboteam_msgs::World world = LastWorld::get();

    if (world.us.size() == 0) {
        std::cout << "No robots, cannot initialize.\n";
    }

    std::vector<int> robots = RobotDealer::get_available_robots();
    int furthest_robot = -1;
    double furthest_robot_x = -std::numeric_limits<double>::max();
    // TODO: Assuming that our goal is left. Either the world should
    // be normalized towards "our goal left" or there should be a bunch
    // of branches here
    for (roboteam_msgs::WorldRobot worldRobot : world.us) {
        if (worldRobot.pos.x > furthest_robot_x) {
            if (std::find(robots.begin(), robots.end(), worldRobot.id) != robots.end()) {
                furthest_robot = worldRobot.id;
                furthest_robot_x = worldRobot.pos.x;
            }
        }
    }

    delete_from_vector(robots, furthest_robot);

    int keeper_robot = -1;
    double keeper_robot_x = std::numeric_limits<double>::max();
    for (roboteam_msgs::WorldRobot worldRobot : world.us) {
        if (worldRobot.pos.x < keeper_robot_x) {
            if (std::find(robots.begin(), robots.end(), worldRobot.id) != robots.end()) {
                keeper_robot = worldRobot.id;
                keeper_robot_x = worldRobot.pos.x;
            }
        }
    }

    delete_from_vector(robots, keeper_robot);

    claim_robots({furthest_robot, keeper_robot});

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", furthest_robot);
        bb.SetBool("GetBall_A_intercept", false);
        bb.SetString("AimAt_A_At", "robot");
        bb.SetInt("AimAt_A_AtRobot", 2);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.node_id = claim_role_node();
        wd.tree = "CoolTree";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        directivePub.publish(wd);
    }

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", keeper_robot);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.node_id = claim_role_node();
        wd.tree = "KeeperBlock";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        directivePub.publish(wd);
    }

    // claim_role_nodes(robot_count);
    // auto allRobots = RobotDealer::get_available_robots();
    // for (int i = 0; i < robot_count; ++i) {
        // int random_index = get_rand(allRobots.size());
        // std::cout << "Claiming " << std::to_string(allRobots.at(random_index)) << "\n";
        // claim_robot(allRobots.at(random_index));
        // allRobots.erase(allRobots.begin() + random_index);
    // }

    // std::cout << "Claimed role nodes: " << get_claimed_role_nodes().size() << "\n"; 

    // int mod = 1;
    // std::string our_field_side = "left";
    // ros::param::get("our_field_side", our_field_side);
    // if (our_field_side == "left") {
        // mod = -1;
    // }

    // int i = 0;
    // auto workers = get_claimed_role_nodes();
    // auto robots = get_claimed_robots();
    // std::random_device rd;
    // std::mt19937 g(rd());
    // std::shuffle(robots.begin(), robots.end(), g);

    // for (auto robot : robots) {
        // // Fill blackboard with relevant info
        // bt::Blackboard bb;
        // bb.SetInt("ROBOT_ID", robot);

        // // Create message
        // roboteam_msgs::RoleDirective wd;
        // wd.node_id = workers.at(i);
        // wd.tree = "GoToPosTree";
        // wd.blackboard = bb.toMsg();

        // // Add random token and save it for later
        // boost::uuids::uuid token = unique_id::fromRandom();
        // tokens.push_back(token);
        // wd.token = unique_id::toMsg(token);

        // // Send to rolenode
        // directivePub.publish(wd);
        // i++;
    // }
}

bt::Node::Status DemoTactic::Update() {
    // bool allSucceeded = true;
    // bool oneFailed = false;
    // bool oneInvalid = false;

    // for (auto token : tokens) {
        // if (feedbacks.find(token) != feedbacks.end()) {
            // Status status = feedbacks.at(token);
            // allSucceeded &= status == bt::Node::Status::Success;
            // oneFailed |= status == bt::Node::Status::Failure;
            // oneInvalid |= status == bt::Node::Status::Invalid;
        // } else {
            // allSucceeded = false;
        // }
    // }

    // if (oneFailed) {
        // return bt::Node::Status::Failure;
    // } else if (oneInvalid) {
        // return bt::Node::Status::Invalid;
    // } else if (allSucceeded) {
        // return bt::Node::Status::Success;
    // }

    return bt::Node::Status::Running;
}

} // rtt
