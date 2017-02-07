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
#include "roboteam_utils/constants.h"

#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/grSim_Replacement.pb.h"

#define RTT_CURRENT_DEBUG_TAG PassToTactic

namespace rtt {

RTT_REGISTER_TACTIC(PassToTactic);

PassToTactic::PassToTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}


void PassToTactic::Initialize(roboteam_utils::Vector2 passToPoint) {
    tokens.clear();
    // RTT_DEBUG("Initializing Tactic\n");

    // This tactic directs two robots
    int PASSER_ID = 1;
    int RECEIVER_ID = 2;
    
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
        bb.SetBool("AimAt_B_setRosParam", false);
        bb.SetString("AimAt_B_At", "robot");
        bb.SetInt("AimAt_B_AtRobot", RECEIVER_ID);

        // Set the kick velocity
        bb.SetDouble("Kick_B_kickVel", 5.0);
        bb.SetBool("Kick_B_wait_for_signal", true);

        // Create message
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

void PassToTactic::ShutdownRoles() {
    passer.tree = passer.STOP_EXECUTING_TREE;
    receiver.tree = receiver.STOP_EXECUTING_TREE;
    pub.publish(passer);
    pub.publish(receiver);
}

bt::Node::Status PassToTactic::Update() {
    // RTT_DEBUG("Updating Tactic\n");
    bool oneFailed = false;
    bool oneInvalid = false;
    bool passerSucces = false;
    bool receiverSucces = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            if (token == unique_id::fromMsg(passer.token) && status == bt::Node::Status::Success) {
                passerSucces = true;
            }

            if (token == unique_id::fromMsg(receiver.token) && status == bt::Node::Status::Success) {
                receiverSucces = true;
            }

            oneFailed |= status == bt::Node::Status::Failure;
            oneInvalid |= status == bt::Node::Status::Invalid;
        }
    }

    if (passerSucces && receiverSucces) {
        ShutdownRoles();
        return bt::Node::Status::Success;
    }

    if (oneFailed) {
        ShutdownRoles();
        return bt::Node::Status::Failure;
    } else if (oneInvalid) {
        return bt::Node::Status::Invalid;
    }

    auto duration = time_difference_milliseconds(start, now());
    if (duration.count() >= 20000) {
        ShutdownRoles();
        return Status::Failure;
    }

    return bt::Node::Status::Running;
}

} // rtt