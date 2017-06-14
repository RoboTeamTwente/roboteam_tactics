#include <sstream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/tactics/BallPlacementUsPlay.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/Vector2.h"

#define RTT_CURRENT_DEBUG_TAG BallPlacementUsPlay

namespace rtt {

RTT_REGISTER_TACTIC(BallPlacementUsPlay);

BallPlacementUsPlay::BallPlacementUsPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void BallPlacementUsPlay::Initialize() {
    tokens.clear();
    
    RTT_DEBUGLN("Preparing to place ball...");

    // auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // auto robots = RobotDealer::get_available_robots();

    // if (RobotDealer::get_keeper_available()) {
        // robots.push_back(RobotDealer::get_keeper());
    // }

    // int KEEPER_ID = RobotDealer::get_keeper();

    // for (auto const ROBOT_ID : robots) {
        // bt::Blackboard bb;
        // bb.SetInt("ROBOT_ID", ROBOT_ID);
        // bb.SetInt("KEEPER_ID", KEEPER_ID);

        // roboteam_msgs::RoleDirective rd;
        // rd.robot_id = ROBOT_ID;
        // rd.tree = "rtt_bob/TwirlTree";
        // rd.blackboard = bb.toMsg();

        // pub.publish(rd);
    // }
}

bt::Node::Status BallPlacementUsPlay::Update() {
    // BallPlacementUsPlay is done when a normal start is signalled.
    // RefStateSwitch will take care of that for us
    
    Vector2 refData = LastRef::get().designated_position;

    std::cout << "Designated position: " << refData << "\n";

    return bt::Node::Status::Running;
}


} // rtt

