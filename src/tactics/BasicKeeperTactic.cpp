#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_tactics/tactics/BasicKeeperTactic.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/Vector2.h"
#include <sstream>
#include "roboteam_utils/Math.h"

#define RTT_CURRENT_DEBUG_TAG BasicKeeperTactic

namespace rtt {

RTT_REGISTER_TACTIC(BasicKeeperTactic);

BasicKeeperTactic::BasicKeeperTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void BasicKeeperTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing\n");
    std::stringstream ss;
    print_blackboard(blackboard, ss);
    RTT_DEBUGLN("%s\n", ss.str().c_str());
    
    // Assign the remaining robots the secondary keeper role
    std::vector<int> robots = getAvailableRobots();

    Vector2 theirGoalPos = LastWorld::get_our_goal_center();
    Vector2 keeperPos(theirGoalPos.x - 0.3*signum(theirGoalPos.x), theirGoalPos.y);

    // Claim the keeper
    const int ROBOT_ID = RobotDealer::get_keeper();
    claim_robots({ROBOT_ID});

    // Fill blackboard with relevant info
    bt::Blackboard bb;

    bb.SetInt("ROBOT_ID", ROBOT_ID);
    bb.SetDouble("Kick_A_kickVel", 2.5);

    bb.SetBool("GetBall_A_intercept", false);
    bb.SetBool("GetBall_A_isKeeper", true);

    bb.SetDouble("GetBall_B_getBallAtX", keeperPos.x);
    bb.SetDouble("GetBall_B_getBallAtY", keeperPos.y);
    bb.SetDouble("GetBall_B_getBallAtTime", 5.0);
    bb.SetBool("GetBall_B_intercept", true);
    bb.SetDouble("GetBall_B_acceptableDeviation", 0.45);
    bb.SetBool("GetBall_B_isKeeper", true);


    // Create message
    roboteam_msgs::RoleDirective wd;
    wd.robot_id = ROBOT_ID;
    wd.tree = "BasicKeeperTree";
    wd.blackboard = bb.toMsg();

    // Send to rolenode
    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    pub.publish(wd);
}

bt::Node::Status BasicKeeperTactic::Update() {
    // Keeper tactic is never done
    return bt::Node::Status::Running;
}


} // rtt

