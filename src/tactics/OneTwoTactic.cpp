#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <random>

#include "unique_id/unique_id.h" 

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/grSim_Replacement.pb.h"

#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_tactics/conditions/IsBallInGoal.h"
#include "roboteam_tactics/tactics/OneTwoTactic.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ComputePassPoint.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG OneTwoTactic

namespace rtt {

RTT_REGISTER_TACTIC(OneTwoTactic);

OneTwoTactic::OneTwoTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}


void OneTwoTactic::Initialize() {
    tokens.clear();
    // RTT_DEBUG("Initializing Tactic\n");

    // This tactic directs two robots
    auto robots = RobotDealer::get_available_robots();

    // Signalling failure should be easier for tactics
    // Either extend the bt framework or give tactic another helper method?
    if (robots.size() < 2) {
        canRun = false;
        return;
    }

    // Just pick the first two
    scorer.robot_id = robots.at(0);
    assistant.robot_id = robots.at(1);

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    
    // Set up the scorer
    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", scorer.robot_id);

        // GetBall_B
        bb.SetString("GetBall_B_AimAt", "robot");
        bb.SetInt("GetBall_B_AimAtRobot", assistant.robot_id);
        bb.SetBool("GetBall_B_AimAtRobotOurTeam", true);

        // Kick_A
        // Needs no other info
        
        // ReceiveBall_B
        // TODO: Not hardcode these coordinates
        bb.SetDouble("ReceiveBall_B_acceptableDeviation", 0.5);
        bb.SetDouble("ReceiveBall_B_receiveBallAtX", 3);
        bb.SetDouble("ReceiveBall_B_receiveBallAtY", 2);

        // AimAt_C
        bb.SetString("At", "robot");
        bb.SetInt("AtRobot", assistant.robot_id);
        
        // Chip_C
        // Needs no other info
        
        // GetBall_D
        bb.SetString("GetDall_And_Look_At_Goal_At", "ourgoal");

        // Create message
        scorer.tree = "rtt_bob/Scorer";
        scorer.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        scorer.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(scorer);
    }

    {
        // Fill blackboard with relevant info
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", assistant.robot_id);

        // ReceiveBall_A
        // TODO: Not hardcode this shit
        bb.SetBool("receiveBallAtPos", false);
        bb.SetDouble("receiveBallAtX", 0);


        // AimAt_A
        
        // Kick_A

        // Create message
        assistant.tree = "Receiver";
        assistant.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        assistant.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(assistant);
    }

    start = rtt::now();
}

void OneTwoTactic::ShutdownRoles() {
    passer.tree = passer.STOP_EXECUTING_TREE;
    receiver.tree = receiver.STOP_EXECUTING_TREE;

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    pub.publish(passer);
    pub.publish(receiver);
}

bt::Node::Status OneTwoTactic::Update() {
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
