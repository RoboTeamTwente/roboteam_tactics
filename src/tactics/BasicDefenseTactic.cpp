#include "roboteam_tactics/tactics/BasicDefenseTactic.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG BasicDefenseTactic

namespace rtt {

BasicDefenseTactic::BasicDefenseTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void BasicDefenseTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing\n");
    
    roboteam_msgs::World world = rtt::LastWorld::get();

    // Assign the remaining robots the secondary keeper role
    std::vector<int> robots = RobotDealer::get_available_robots();

    auto& pub = rtt::get_roledirective_publisher();

    for (const int ROBOT_ID : robots) {
        // Fill blackboard with relevant info
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", ROBOT_ID);

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = ROBOT_ID;
        wd.tree = "GoToPos";
        wd.blackboard = bb.toMsg();

        // Send to rolenode
        pub.publish(wd);

        RTT_DEBUG("Claiming: %i\n", ROBOT_ID);
    }

    claim_robots(robots);
}

bt::Node::Status BasicDefenseTactic::Update() {
    // Defense tactic is never done
    return bt::Node::Status::Running;
}


} // rtt

