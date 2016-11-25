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
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG DemoTactic

namespace rtt {

DemoTactic::DemoTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

template<typename T>
void delete_from_vector(std::vector<T> &items, const T &item) {
    auto it = std::find(items.begin(), items.end(), item);
    if (it != items.end()) {
        items.erase(it);
    }
}

int get_most_attacking_robot(std::vector<int> robots, const roboteam_msgs::World &world) {
    int side = LastWorld::get_our_goal_center().x;

    int furthest_robot = -1;
    double furthest_robot_side_dist = -std::numeric_limits<double>::max();
    // TODO: Assuming that our goal is left. Either the world should
    // be normalized towards "our goal left" or there should be a bunch
    // of branches here
    for (roboteam_msgs::WorldRobot worldRobot : world.us) {
        double this_robot_side_dist = std::abs(worldRobot.pos.x - side);
        if (this_robot_side_dist > furthest_robot_side_dist) {
            if (std::find(robots.begin(), robots.end(), worldRobot.id) != robots.end()) {
                furthest_robot = worldRobot.id;
                furthest_robot_side_dist = this_robot_side_dist;
            }
        }
    }

    return furthest_robot;
}

int get_robot_closest_to_ball(std::vector<int> robots, const roboteam_msgs::World &world) {
    int closest_robot = -1;
    double closest_robot_ds = std::numeric_limits<double>::max();

    // TODO: Why is there not a copy constructor?
    roboteam_utils::Vector2 ball_pos(world.ball.pos);

    for (roboteam_msgs::WorldRobot worldRobot : world.us) {
        roboteam_utils::Vector2 pos(worldRobot.pos);

        if ((pos - ball_pos).length() < closest_robot_ds) {
            if (std::find(robots.begin(), robots.end(), worldRobot.id) != robots.end()) {
                closest_robot = worldRobot.id;
                closest_robot_ds = (pos - ball_pos).length();
            }
        }
    }

    return closest_robot;
}

void DemoTactic::Initialize() {
    tokens.clear();

    std::cout << "Initializing demo tactic";

    roboteam_msgs::World world = LastWorld::get();

    if (RobotDealer::get_available_robots().size() < 4) {
        std::cout << "Not enough robots, cannot initialize.\n";
        // TODO: Want to pass failure here as well!
        return;
    }

    std::vector<int> robots = RobotDealer::get_available_robots();

    int attack_bot = get_robot_closest_to_ball(robots, world);
    delete_from_vector(robots, attack_bot);

    int score_bot = get_most_attacking_robot(robots, world);
    delete_from_vector(robots, score_bot);

    int def_bot = robots.back();
    delete_from_vector(robots, def_bot);

    int keeper_bot = RobotDealer::get_keeper();

    claim_robots({score_bot, attack_bot, def_bot, keeper_bot});

    int mod = -1;
    if (rtt::get_our_field_side() == "left") {
        mod = 1;
    }

    roboteam_utils::Vector2 passTo(3 * mod, get_rand_int(6000) / 6000.0 * 6 - 3);

    RTT_DEBUG("Attack bot: %i, score bot: %i, keeper bot: %i\n", attack_bot, score_bot, keeper_bot);

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", score_bot);
        bb.SetBool("GetBall_A_intercept", true);
        bb.SetDouble("GetBall_A_getBallAtX", passTo.x);
        bb.SetDouble("GetBall_A_getBallAtY", passTo.y);
        bb.SetDouble("GetBall_A_getBallAtTime", 5.0);
        bb.SetString("AimAt_A_At", "theirgoal");

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = score_bot;
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
        bb.SetInt("ROBOT_ID", attack_bot);
        bb.SetBool("GetBall_A_intercept", false);
        bb.SetString("AimAt_A_At", "robot");
        bb.SetInt("AimAt_A_AtRobot", score_bot);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = attack_bot;
        wd.tree = "CoolTree";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        directivePub.publish(wd);
    }

    // {
        // // Fill blackboard with relevant info
        // bt::Blackboard bb;
        // bb.SetInt("ROBOT_ID", def_bot);

        // // Create message
        // roboteam_msgs::RoleDirective wd;
        // wd.robot_id = def_bot;
        // wd.tree = "SecondaryKeeper";
        // wd.blackboard = bb.toMsg();

        // // Add random token and save it for later
        // boost::uuids::uuid token = unique_id::fromRandom();
        // wd.token = unique_id::toMsg(token);

        // // Send to rolenode
        // // directivePub.publish(wd);
    // }

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", keeper_bot);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = keeper_bot;
        wd.tree = "BasicKeeperTree";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        directivePub.publish(wd);
    }

    start = rtt::now();
}

bt::Node::Status DemoTactic::Update() {
    bool allSucceeded = true;
    bool oneFailed = false;
    bool oneInvalid = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            allSucceeded &= status == bt::Node::Status::Success;
            oneFailed |= status == bt::Node::Status::Failure;
            oneInvalid |= status == bt::Node::Status::Invalid;
        } else {
            allSucceeded = false;
        }
    }

    if (oneFailed) {
        return bt::Node::Status::Failure;
    } else if (oneInvalid) {
        return bt::Node::Status::Invalid;
    } else if (allSucceeded) {
        return bt::Node::Status::Success;
    }

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 5) {
        return Status::Failure;
    }

    return bt::Node::Status::Running;
}

} // rtt
