#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/AttackerTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/LastWorld.h"

namespace rtt {

AttackerTactic::AttackerTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

template<typename T>
void delete_from_vector(std::vector<T> &items, const T &item) {
    auto it = std::find(items.begin(), items.end(), item);
    if (it != items.end()) {
        items.erase(it);
    }
}


void AttackerTactic::Initialize() {
    tokens.clear();

    std::cout << "Initializing attacker tactic";
    
    if (RobotDealer::get_available_robots().size() < 4) {
        std::cout << "Not enough robots, cannot initialize.\n";
        // TODO: Want to pass failure here as well!
        return;
    }


    roboteam_msgs::World world = rtt::LastWorld::get();
    roboteam_utils::Vector2 ballPos(world.ball.pos.x, world.ball.pos.y);

    // This tactic directs two robots
    std::array<int, 2> availableRobots = {0, 1};
    roboteam_utils::Vector2 firstRobotPos(world.us.at(availableRobots.at(0)).pos.x, world.us.at(availableRobots.at(0)).pos.y);
    roboteam_utils::Vector2 secondRobotPos(world.us.at(availableRobots.at(1)).pos.x, world.us.at(availableRobots.at(1)).pos.y);
    
    int primaryAttacker;
    int secondaryAttacker;

    // Assign the role of primary attacker to the robot that is closest to the ball
    if ((firstRobotPos - ballPos).length() < (secondRobotPos - ballPos).length()) {
        primaryAttacker = availableRobots.at(0);
        secondaryAttacker = availableRobots.at(1);
    } else {
        primaryAttacker = availableRobots.at(1);
        secondaryAttacker = availableRobots.at(0);
    }

    std::vector<int> robots = RobotDealer::get_available_robots();

    delete_from_vector(robots, primaryAttacker);
    delete_from_vector(robots, secondaryAttacker);

    // int def_bot = robots.back();
    // delete_from_vector(robots, def_bot);

    int keeper_bot = robots.back();
    delete_from_vector(robots, keeper_bot);

    // claim_robots({keeper_bot});
    claim_robots({primaryAttacker, secondaryAttacker, keeper_bot});


    ROS_INFO_STREAM("primaryAttacker: " << primaryAttacker << " secondaryAttacker: " << secondaryAttacker);

    {

        // Fill blackboard with relevant info
        bt::Blackboard bb;
        // Attacker 1
        bb.SetInt("ROBOT_ID", primaryAttacker);

        // Get the ball!
        bb.SetBool("GetBall_A_intercept", false);
        
        // If you can see the goal, aim towards it
        bb.SetBool("AimAt_A_setRosParam", false);
        bb.SetString("AimAt_A_At", "theirgoal");

        // Else, if you can see the other attacker, aim to him
        bb.SetBool("CanSeeRobot_A_our_team", true);
        bb.SetInt("CanSeeRobot_A_targetID", secondaryAttacker);
        bb.SetBool("AimAt_B_setRosParam", true);
        bb.SetString("AimAt_B_At", "robot");
        bb.SetInt("AimAt_B_AtRobot", secondaryAttacker);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = primaryAttacker;
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
        // Attacker 2
        bb.SetInt("ROBOT_ID", secondaryAttacker);

        // Make sure you stand free to receive the ball
        bb.SetInt("StandFree_A_theirID", primaryAttacker);
        bb.SetBool("StandFree_A_setRosParam", true);
        bb.SetString("StandFree_A_whichTeam", "us");
        bb.SetDouble("StandFree_A_distanceFromPoint", 0.3);

        // Receive the ball
        bb.SetBool("GetBall_A_intercept", true);
        // bb.SetDouble("GetBall_A_getBallAtX", 0.0); // these positions will be updated in the world callback to match the robot's current position
        // bb.SetDouble("GetBall_A_getBallAtY", 0.0);
        bb.SetBool("GetBall_A_getBallAtCurrentPos", true);

        // Aim at goal
        bb.SetBool("AimAt_A_setRosParam", false);
        bb.SetString("AimAt_A_At", "theirgoal");

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = secondaryAttacker;
        wd.tree = "SuperCoolTree";
        wd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);

        // Send to rolenode
        directivePub.publish(wd);
    }

    // {
    //     // Fill blackboard with relevant info
    //     bt::Blackboard bb;
    //     bb.SetInt("ROBOT_ID", def_bot);

    //     // Create message
    //     roboteam_msgs::RoleDirective wd;
    //     wd.robot_id = def_bot;
    //     wd.tree = "SecondaryKeeper";
    //     wd.blackboard = bb.toMsg();

    //     // Add random token and save it for later
    //     boost::uuids::uuid token = unique_id::fromRandom();
    //     wd.token = unique_id::toMsg(token);

    //     // Send to rolenode
    //     directivePub.publish(wd);
    // }

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", keeper_bot);

        bb.SetBool("GetBall_A_intercept", false);
        bb.SetString("AimAt_A_At", "robot");
        bb.SetInt("AimAt_A_AtRobot", primaryAttacker);

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

bt::Node::Status AttackerTactic::Update() {
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
