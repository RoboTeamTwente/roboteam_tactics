#include "roboteam_tactics/tactics/BasicKeeperTactic.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG BasicKeeperTactic

namespace rtt {

RTT_REGISTER_TACTIC(BasicKeeperTactic);

BasicKeeperTactic::BasicKeeperTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void BasicKeeperTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing\n");
    
    // Assign the remaining robots the secondary keeper role
    std::vector<int> robots = RobotDealer::get_available_robots();

    // Claim the keeper
    const int ROBOT_ID = RobotDealer::get_keeper();
    claim_robots({ROBOT_ID});

    // Fill blackboard with relevant info
    bt::Blackboard bb;

    bb.SetInt("ROBOT_ID", ROBOT_ID);

    // Create message
    roboteam_msgs::RoleDirective wd;
    wd.robot_id = ROBOT_ID;
    wd.tree = "BasicKeeperTree";
    wd.blackboard = bb.toMsg();

    // Send to rolenode
    // rtt::get_roledirective_publisher().publish(wd);
    ros::NodeHandle n;
    auto pub = n.advertise<roboteam_msgs::RoleDirective>("role_directive", 100);
    pub.publish(wd);
}

bt::Node::Status BasicKeeperTactic::Update() {
    // Keeper tactic is never done
    return bt::Node::Status::Running;
}


} // rtt

