#include "roboteam_tactics/tactics/FreeKickDefenceTactic.h"

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/SkillFactory.h"
#include "roboteam_tactics/treegen/LeafRegister.h"  
#include "roboteam_tactics/skills/Block.h"  
#include "ros/ros.h"

#define RTT_CURRENT_DEBUG_TAG FreeKickDefenceTactic

namespace rtt {

/*
 * Plan:
 * - Keeper blocks path between ball and goal.
 * - 1 bot blocks path between most dangerous opponent, or second most dangerous if the most dangerous is
 *   the one taking the free kick.
 * - 1 bot harasses the free kick-taker by blocking them (at a legal distance).
 * - OPTIONAL: 1/2 bots block paths to other opponents, selecting the most dangerous.
 */
    
RTT_REGISTER_TACTIC(FreeKickDefenceTactic);

FreeKickDefenceTactic::FreeKickDefenceTactic(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard) {
    isInitialized = false;
}

void FreeKickDefenceTactic::Initialize() {
    if (!danger_finder.is_running()) {
        danger_finder.run();
    }
    tokens.clear();
    ros::Duration(.1).sleep();
    RTT_DEBUGLN("Initializing FreeKickDefence");
    std::vector<int> available = RobotDealer::get_available_robots();
    roboteam_msgs::World world = LastWorld::get();
    bool has_keeper = RobotDealer::get_keeper_available();
    
    if (available.size() < 2) {
        RTT_DEBUGLN("Only %d bots available, need 2+", (int) available.size());
        return;
    }
    if (!has_keeper) {
        RTT_DEBUGLN("Keeper not available. (Free kick should probably have priority access to the keeper?)");
        return;
    }
    int additional = std::min((int) available.size() - 2, 2);
    
    RTT_DEBUGLN("Running FreeKickDefence with %d additional bots", additional);
    
    int keeper = RobotDealer::get_keeper();
    int defender, harasser, interceptor1 = -1, interceptor2 = -1;
    
    RTT_DEBUGLN("Self:");
    for (auto opp : world.us) {
        RTT_DEBUGLN("\t#%d @ (%f, %f)", opp.id, opp.pos.x, opp.pos.y);
    }
    RTT_DEBUGLN("Opponents:");
    for (auto opp : world.them) {
        RTT_DEBUGLN("\t#%d @ (%f, %f)", opp.id, opp.pos.x, opp.pos.y);
    }
    
    // Set harasser: blocks kick taker
    auto free_kick_taker_opt = getBallHolder();
    if (!free_kick_taker_opt) {
        RTT_DEBUGLN("Cannot find free kick taker... Try again later?");
        return;
    }
    auto free_kick_taker = free_kick_taker_opt->first;
    harasser = get_robot_closest_to_point(available, world, free_kick_taker.pos);
    {
        auto it = std::find(available.begin(), available.end(), harasser);
        assert(it != available.end());
        available.erase(it);
    }
    
    // Set defender: blocks most dangerous opponent
    auto most_dangerous_opt = danger_finder.get_immediate_update().most_dangerous;
    if (!most_dangerous_opt) {
        RTT_DEBUGLN("Cannot find most dangerous bot...");
        return;
    }
    auto most_dangerous = *most_dangerous_opt;
    if (most_dangerous.id == free_kick_taker.id) {
        most_dangerous = *(danger_finder.current_result().second_most_dangerous);
    }
    defender = get_robot_closest_to_point(available, world, most_dangerous.pos);
    {
        auto it = std::find(available.begin(), available.end(), defender);
        (it != available.end());
        available.erase(it);
    }
    
    // Get additional targets (randomly for now) and assign interceptors
    roboteam_msgs::WorldRobot additional_tgt_1, additional_tgt_2;
    if (additional > 0) {
        do {
            additional_tgt_1 = *get_rand_element_seq<roboteam_msgs::WorldRobot>(world.them);
        } while (additional_tgt_1.id == free_kick_taker.id || additional_tgt_1.id == most_dangerous.id);
        interceptor1 = get_robot_closest_to_point(available, world, additional_tgt_1.pos);
        auto it = std::find(available.begin(), available.end(), interceptor1);
        assert(it != available.end());
        available.erase(it);
    } 
    if (additional > 1) {
        do {
            additional_tgt_2 = *get_rand_element_seq<roboteam_msgs::WorldRobot>(world.them);
        } while (additional_tgt_2.id == free_kick_taker.id || additional_tgt_2.id == most_dangerous.id
                 || additional_tgt_2.id == additional_tgt_1.id);
        interceptor2 = get_robot_closest_to_point(available, world, additional_tgt_2.pos);
        auto it = std::find(available.begin(), available.end(), interceptor2);
        assert(it != available.end());
        available.erase(it);
    }    
    
    RTT_DEBUGLN("Assignments: D=%d->%d H=%d->%d I1=%d->%d I2=%d->%d",
                defender, most_dangerous.id,
                harasser, free_kick_taker.id,
                interceptor1, (additional > 0 ? additional_tgt_1.id : -1),
                interceptor2, (additional > 0 ? additional_tgt_2.id : -1));
    
    isInitialized = true;
    
    auto pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    {
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", defender);
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = defender;
        wd.tree = "SecondaryKeeperTree";
        wd.blackboard = bb.toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
    
    {
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", harasser);
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = defender;
        wd.tree = "HarasserTree";
        wd.blackboard = bb.toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
    
    if (additional > 0) {
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", interceptor1);
        bb.SetInt("TGT_ID", additional_tgt_1.id);
        bb.SetInt("IHaveBall_A_me", additional_tgt_1.id);
        bb.SetBool("IHaveBall_A_our_team", false);
        bb.SetString("Block_A_block_type", "RELATIVE");
        bb.SetDouble("Block_A_block_arg", 0.5);
        bb.SetBool("Block_A_invert_direction", true);
        bb.SetInt("Block_A_BLOCK_ID", BLOCK_BALL_ID);
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = interceptor1;
        wd.tree = "InterceptorTree";
        wd.blackboard = bb.toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
    
    if (additional > 1) {
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", interceptor2);
        bb.SetInt("TGT_ID", additional_tgt_2.id);
        bb.SetInt("IHaveBall_A_me", additional_tgt_2.id);
        bb.SetBool("IHaveBall_A_our_team", false);
        bb.SetString("Block_A_block_type", "RELATIVE");
        bb.SetDouble("Block_A_block_arg", 0.5);
        bb.SetBool("Block_A_invert_direction", true);
        bb.SetInt("Block_A_BLOCK_ID", BLOCK_BALL_ID);
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = interceptor2;
        wd.tree = "InterceptorTree";
        wd.blackboard = bb.toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
    
}

bt::Node::Status FreeKickDefenceTactic::Update() {
    return bt::Node::Status::Running;
}

}