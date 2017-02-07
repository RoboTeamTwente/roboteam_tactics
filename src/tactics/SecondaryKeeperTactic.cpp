#include "roboteam_tactics/tactics/SecondaryKeeperTactic.h"

#define RTT_CURRENT_DEBUG_TAG SecondaryKeeperTactic

namespace rtt {
    
void SecondaryKeeperTactic::Initialize() {
    auto world = LastWorld::get();
    auto bots = RobotDealer::get_available_robots();
    
    if (bots.size() == 0) {
        RTT_DEBUGLN("No robots available for SecondaryKeeperTactic...");
        return;
    }
    
    int bot = get_robot_closest_to_our_goal(bots);
    assert(bot > -1);
    claim_robot(bot);
    
    {
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", bot);
        
        roboteam_msgs::RoleDirective dir;
        dir.robot_id = bot;
        dir.tree = "SecondaryKeeperTree"; //TODO
        dir.blackboard = bb.toMsg();
        rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher().publish(dir);
    }
}

bt::Node::Status SecondaryKeeperTactic::Update() {
    return bt::Node::Status::Running;
}
    
}