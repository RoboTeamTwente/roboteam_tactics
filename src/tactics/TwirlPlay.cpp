#include <sstream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/tactics/TwirlPlay.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_msgs/RobotCommand.h"

#define RTT_CURRENT_DEBUG_TAG TwirlPlay

namespace rtt {

RTT_REGISTER_TACTIC(TwirlPlay);

TwirlPlay::TwirlPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void TwirlPlay::Initialize() {
    tokens.clear();
    
    RTT_DEBUGLN("Preparing Kickoff...");

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    auto robots = RobotDealer::get_available_robots();

    if (RobotDealer::get_keeper_available()) {
        robots.push_back(RobotDealer::get_keeper());
    }

    int KEEPER_ID = RobotDealer::get_keeper();

    for (auto const ROBOT_ID : robots) {
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", ROBOT_ID);
        bb.SetInt("KEEPER_ID", KEEPER_ID);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = ROBOT_ID;
        rd.tree = "rtt_bob/TwirlTree";
        rd.blackboard = bb.toMsg();

        pub.publish(rd);
    }
}

bt::Node::Status TwirlPlay::Update() {
    // TwirlPlay is done when a normal start is signalled.
    // RefStateSwitch will take care of that for us
    return bt::Node::Status::Running;
}


} // rtt

