#include "roboteam_tactics/tactics/TwoVTwoDefenseTactic.h"

#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#define RTT_CURRENT_DEBUG_TAG TwoVTwoDefenseTactic

namespace rtt {

RTT_REGISTER_TACTIC(TwoVTwoDefenseTactic);
    
TwoVTwoDefenseTactic::TwoVTwoDefenseTactic(std::string name, bt::Blackboard::Ptr blackboard) 
    : Tactic(name, blackboard) {}
    

void TwoVTwoDefenseTactic::Initialize() {
    tokens.clear();
    auto robots = RobotDealer::get_available_robots();
    if (robots.size() < 2) {
        canRun = false;
        return;
    }
    
    keeper.robot_id = robots[0];
    harasser.robot_id = robots[1];
    RTT_DEBUGLN("Keeper=%d, harasser=%d", keeper.robot_id, harasser.robot_id);
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    
    {
        Vector2 theirGoalPos = LastWorld::get_our_goal_center();
        Vector2 keeperPos(theirGoalPos.x - 0.3*signum(theirGoalPos.x), theirGoalPos.y);
    
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", keeper.robot_id);
        bb.SetInt("KEEPER_ID", keeper.robot_id);
        bb.SetDouble("Kick_A_kickVel", 2.5);

        ScopedBB(bb, "GetBall_A")
            .setBool("intercept", false)
            .setBool("isKeeper", true);

        ScopedBB(bb, "GetBall_B")
            .setDouble("getBallAtX", keeperPos.x)
            .setDouble("getBallAtY", keeperPos.y)
            .setDouble("getBallAtTime", 5.0)
            .setBool("intercept", true)
            .setDouble("acceptableDeviation", .45)
            .setBool("isKeeper", true);
            
        keeper.tree = "BasicKeeperTree";
        keeper.blackboard = bb.toMsg();
        
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        keeper.token = unique_id::toMsg(token);

        pub.publish(keeper);
    }
    
    {
        auto holder = getBallHolder();
        int tgt;
        if (holder && !holder->second) {
            tgt = holder->first.id;
        } else {
            tgt = getBotFromDangerList(0)->id;
        }
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", harasser.robot_id);
        bb.SetInt("KEEPER_ID", keeper.robot_id);
        bb.SetInt("TGT_ID", tgt);
        /*
        ScopedBB(bb, "Harass_A_Block_Kick")
            .setString("block_type", "COVER")
            .setBool("invert_direction", true)
            .setInt("BLOCK_ID", -1);
        
        ScopedBB(bb, "Harass_A_Block_Get")
            .setString("block_type", "COVER")
            .setBool("invert_direction", false)
            .setInt("BLOCK_ID", -1);
        */
        harasser.tree = "HarasserTree";
        harasser.blackboard = bb.toMsg();
        
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        harasser.token = unique_id::toMsg(token);

        pub.publish(harasser);
    }
}

bt::Node::Status TwoVTwoDefenseTactic::Update() {
    return bt::Node::Status::Running;
}
    
}

#undef RTT_CURRENT_DEBUG_TAG
