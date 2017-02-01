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
    RTT_DEBUG("Initializing Tactic\n");

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


        // ParamSet_A
        // Set in tree
        // TODO: Resetting stale values should not be needed
        
        // EnemyCloseToBall_B
        // Set in tree
        
        // GetBall_C
        bb.SetString("GetBall_C_AimAt", "robot");
        bb.SetInt("GetBall_C_AimAtRobot", assistant.robot_id);
        bb.SetBool("GetBall_C_AimAtRobotOurTeam", true);

        // Kick_C
        // Needs no other info

        // GoToPos_D
        // TODO: Not hardcode these coordinates
        bb.SetDouble("GoToPos_D_xGoal", 3);
        bb.SetDouble("GoToPos_D_yGoal", 2);
        bb.SetDouble("GoToPos_D_angleGoal", M_PI * 1.25);
        bb.SetBool("GoToPos_D_dribbler", true);
        
        // ParamSet_D
        // Set in tree
        
        // ReceiveBall_D
        bb.SetDouble("ReceiveBall_D_acceptableDeviation", 0.5);
        bb.SetBool("ReceiveBall_D_receiveBallAtCurrentPos", true);

        // AimAt_D
        bb.SetString("AimAt_D_At", "ourgoal");
        
        // Chip_D
        // Needs no other info
        
        // GetBall_And_Look_At_Goal
        bb.SetString("GetBall_And_Look_At_Goal_At", "ourgoal");

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
        bb.SetBool("ReceiveBall_A_receiveBallAtCurrentPos", false);
        bb.SetDouble("ReceiveBall_A_receiveBallAtX", 0);
        bb.SetDouble("ReceiveBall_A_receiveBallAtY", 0);

        // ParamCheck_B
        
        // AimAt_B
        bb.SetString("AimAt_A_At", "robot");
        bb.SetInt("AimAt_A_AtRobot", scorer.robot_id);
        
        // Kick_C
        // No extra info here!

        // Create message
        assistant.tree = "rtt_bob/Assistant";
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
    scorer.tree = roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE;
    assistant.tree = roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE;

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    pub.publish(scorer);
    pub.publish(assistant);
}

bt::Node::Status OneTwoTactic::Update() {
    // RTT_DEBUGLN("Updating Tactic");
    bool oneFailed = false;
    bool oneInvalid = false;
    bool scorerSuccess = false;
    bool assistantSuccesss = false;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            if (token == unique_id::fromMsg(scorer.token) && status == bt::Node::Status::Success) {
                scorerSuccess = true;
            }

            if (token == unique_id::fromMsg(assistant.token) && status == bt::Node::Status::Success) {
                assistantSuccesss = true;
            }

            oneFailed |= status == bt::Node::Status::Failure;
            oneInvalid |= status == bt::Node::Status::Invalid;
        }
    }

    if (scorerSuccess && assistantSuccesss) {
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