#include <sstream>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/tactics/Bob_PenaltyShootOutPlay.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_msgs/RobotCommand.h"

// #define RTT_CURRENT_DEBUG_TAG Bob_PenaltyShootOutPlay

namespace rtt {

RTT_REGISTER_TACTIC(Bob_PenaltyShootOutPlay);

Bob_PenaltyShootOutPlay::Bob_PenaltyShootOutPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void Bob_PenaltyShootOutPlay::Initialize() {
    tokens.clear();
    
    // RTT_DEBUGLN("Preparing twirling...");

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    auto robots = getAvailableRobots();

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

bt::Node::Status Bob_PenaltyShootOutPlay::Update() {
    // Bob_PenaltyShootOutPlay is done when a normal start is signalled.
    // RefStateSwitch will take care of that for us
    return bt::Node::Status::Running;
}


} // rtt

