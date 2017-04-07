#include <tuple>
#include <boost/uuid/uuid.hpp>
namespace b = boost;

#include "roboteam_tactics/tactics/rtt_bob/NaiveGetBallWithSupportPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/ScopedBB.h"

namespace rtt {

namespace rtt_bob {

RTT_REGISTER_TACTIC_F(rtt_bob, NaiveGetBallWithSupportPlay);

NaiveGetBallWithSupportPlay::NaiveGetBallWithSupportPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

roboteam_msgs::RoleDirective NaiveGetBallWithSupportPlay::initSupport(
        int KEEPER_ID, 
        int ROBOT_ID, 
        int SPITS_ID, 
        int OTHER_SUPPORT,
        bool top
        ) {

    double waitPosX = 4.5 / 2, 
           waitPosY;
    if (top) {
        waitPosY = 1.5;
    } else {
        waitPosY = -1.5;
    }
    
    bt::Blackboard bb;
    bb.SetInt("ROBOT_ID", ROBOT_ID);
    bb.SetInt("KEEPER_ID", KEEPER_ID);

    ScopedBB(bb, "ReceiveBall_")
        .setDouble("receiveBallAtX", waitPosX)
        .setDouble("receiveBallAtY", waitPosY)
        .setDouble("acceptableDeviation", 0.5)
        ;

    ScopedBB(bb, "CanSeeGoal_")
        .setDouble("x_coor", 0)
        .setDouble("y_coor", 4.5)
        ;

    ScopedBB(bb, "AimAt_Goal")
        .SetString("At", "theirgoal")
        ;

    ScopedBB(bb, "CanSeeSpits")
        .SetInt("targetID", SPITS_ID)
        ;

    ScopedBB(bb, "AimAt_Spits_A")
        .SetString("At", "robot")
        .SetInt("AtRobot", SPITS_ID)
        ;

    roboteam_msgs::RoleDirective rd;
    rd.robot_id = ROBOT_ID;
    rd.tree = "rtt_bob/TwirlTree";
    rd.blackboard = bb.toMsg();

}

roboteam_msgs::RoleDirective NaiveGetBallWithSupportPlay::initSpits() {

}

void NaiveGetBallWithSupportPlay::Initialize() {
    // Pick spits
    // Pick support1
    // Pick suport2
    
    // Fail if robots cannot be picked
    
    // initialise spits
    // initialise support1
    // initialise support2

    return;
}

bt::Node::Status NaiveGetBallWithSupportPlay::Update() {
    // Fail if robots could not be picked
    
    return Status::Running;
}

} // rtt_bob

} // rtt

