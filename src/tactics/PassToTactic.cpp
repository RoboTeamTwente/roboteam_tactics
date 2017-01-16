#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/PassToTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/ComputePassPoint.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/conditions/IsBallInGoal.h"
#include "roboteam_utils/Vector2.h"

#define RTT_CURRENT_DEBUG_TAG PassToTactic

namespace rtt {

RTT_REGISTER_TACTIC(PassToTactic);

PassToTactic::PassToTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void PassToTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing\n");
    
    if (RobotDealer::get_available_robots().size() < 4) {
        RTT_DEBUG("Not enough robots, cannot initialize.\n");
        // TODO: Want to pass failure here as well!
        return;
    }

    roboteam_msgs::World world = rtt::LastWorld::get();

    roboteam_utils::Vector2 passToPoint(2.0, 0.0);

    // This tactic directs two robots
    int PASSER_ID = 0;
    int RECEIVER_ID = 1;

    std::vector<int> robots = RobotDealer::get_available_robots();
    
    delete_from_vector(robots, PASSER_ID);
    delete_from_vector(robots, RECEIVER_ID);

    claim_robots({PASSER_ID, RECEIVER_ID});

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        // Attacker 1
        bb.SetInt("ROBOT_ID", PASSER_ID);

        // Get the ball!
        bb.SetBool("GetBall_B_intercept", false);
        bb.SetString("GetBall_B_AimAt", "robot");
        bb.SetInt("GetBall_B_AimAtRobot", RECEIVER_ID);
        bb.SetBool("GetBall_B_AimAtRobotOurTeam", true);
        bb.SetBool("GetBall_B_isKeeper", false);

        // Check if you can see the other attacker, and aim to him
        bb.SetBool("CanSeeRobot_B_our_team", true);
        bb.SetInt("CanSeeRobot_B_targetID", RECEIVER_ID);
        bb.SetBool("AimAt_B_setRosParam", true);
        bb.SetString("AimAt_B_At", "robot");
        bb.SetInt("AimAt_B_AtRobot", RECEIVER_ID);

        // Set the kick velocity
        bb.SetDouble("Kick_B_kickVel", 5.0);

        // Create message
        roboteam_msgs::RoleDirective passer;
        passer.robot_id = PASSER_ID;
        passer.tree = "Passer";
        passer.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        passer.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(passer);
    }

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", RECEIVER_ID);

        // Receive the ball
        bb.SetBool("GetBall_B_intercept", true);
        bb.SetBool("GetBall_B_getBallAtCurrentPos", false);
        bb.SetDouble("GetBall_B_getBallAtX", passToPoint.x);
        bb.SetDouble("GetBall_B_getBallAtY", passToPoint.y);
        bb.SetDouble("GetBall_B_getBallAtTime", 10.0);

        // Create message
        roboteam_msgs::RoleDirective receiver;
        receiver.robot_id = RECEIVER_ID;
        receiver.tree = "Receiver";
        receiver.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        receiver.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(receiver);
    }

    start = rtt::now();
}

bt::Node::Status PassToTactic::Update() {
    bool allSucceeded = true;
    // bool oneSucceeded = false;
    bool oneFailed = false;
    bool oneInvalid = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);

            if (status == bt::Node::Status::Success) {
                // ROS_INFO("something succeeded!");
            }

            allSucceeded &= status == bt::Node::Status::Success;
            // oneSucceeded |= status == bt::Node::Status::Success;
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
        ROS_INFO("yay, all succeeded!");
        return bt::Node::Status::Success;
    }

    auto duration = time_difference_seconds(start, now());
    // if (duration.count() >= 8) {
    //     return Status::Failure;
    // }

    return bt::Node::Status::Running;
}

} // rtt
