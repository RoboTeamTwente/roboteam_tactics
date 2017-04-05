#include "roboteam_tactics/tactics/Qualification1v1Tactic.h"

#define RTT_CURRENT_DEBUG_TAG Qualification1v1Tactic
    
namespace rtt {
    
RTT_REGISTER_TACTIC(Qualification1v1Tactic);

Qualification1v1Tactic::Qualification1v1Tactic(std::string name, bt::Blackboard::Ptr bb)
    : Tactic(name, bb), canRun(true) {}
    
void Qualification1v1Tactic::Initialize() {
    RTT_DEBUGLN("Q1v1 Initializing");
    tokens.clear();
    auto robots = RobotDealer::get_available_robots();
    if (robots.size() < 2) {
        canRun = false;
        return;
    }
    
    firstRD.robot_id = robots.at(0);
    secondRD.robot_id = robots.at(1);
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    
    {
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", robots.at(0));
        bb.SetInt("KEEPER_ID", -1);
        bb.SetInt("AimAtRobot", robots.at(1));
        // Everything else set in tree
        
        firstRD.tree = "qualification/Qualification1v1FirstRole";
        firstRD.blackboard = bb.toMsg();
        
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        firstRD.token = unique_id::toMsg(token);

        pub.publish(firstRD);
    }
    {
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", robots.at(1));
        bb.SetInt("KEEPER_ID", -1);
        bb.SetInt("Block_A_TGT_ID", robots.at(0));
        bb.SetInt("IHaveBall_A_me", robots.at(0));
        bb.SetBool("IHaveBall_A_our_team", true);
        bb.SetInt("IHaveBall_B_me", robots.at(1));
        bb.SetBool("IHaveBall_B_our_team", true);
        
        
        secondRD.tree = "qualification/Qualification1v1SecondRole";
        secondRD.blackboard = bb.toMsg();
        
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        secondRD.token = unique_id::toMsg(token);

        pub.publish(secondRD);
    }
    RTT_DEBUGLN("Q1v1 Initialized");
}

bt::Node::Status Qualification1v1Tactic::Update() {
    if (!canRun) {
        return bt::Node::Status::Invalid;
    }
    
    bool error = false, failure = false, success = true;
    
    if (feedbacks.find(tokens.at(0)) != feedbacks.end()) {
        auto stat = feedbacks.at(tokens.at(0));
        RTT_DEBUGLN("First Status: %d", static_cast<int>(stat));
        error |= stat == bt::Node::Status::Invalid;
        failure |= stat == bt::Node::Status::Failure;
        success &= stat == bt::Node::Status::Success;
    } else success = false;
    
    if (feedbacks.find(tokens.at(1)) != feedbacks.end()) {
        auto stat = feedbacks.at(tokens.at(1));
        RTT_DEBUGLN("Second Status: %d", static_cast<int>(stat));
        error |= stat == bt::Node::Status::Invalid;
        failure |= stat == bt::Node::Status::Failure;
        success &= stat == bt::Node::Status::Success;
    } else success = false;
    
    if (error || failure || success) {
        firstRD.tree = firstRD.STOP_EXECUTING_TREE;
        secondRD.tree = secondRD.STOP_EXECUTING_TREE;
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
        pub.publish(firstRD);
        pub.publish(secondRD);
        return error ? bt::Node::Status::Invalid : (failure ? bt::Node::Status::Failure : bt::Node::Status::Success);
    }
    return bt::Node::Status::Running;
}

}