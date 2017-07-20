#include "roboteam_tactics/tactics/InterceptorsTactic.h"

#define RTT_CURRENT_DEBUG_TAG InterceptorsTactic

namespace rtt {
    
RTT_REGISTER_TACTIC(InterceptorsTactic);
    
void InterceptorsTactic::assign(int own, const roboteam_msgs::WorldRobot& opp) {
    bt::Blackboard bb;
    bb.SetInt("ROBOT_ID", own);
    bb.SetInt("IHaveBall_A_me", opp.id);
    bb.SetBool("IHaveBall_A_our_team", false);
    bb.SetInt("Block_A_TGT_ID", opp.id);
    bb.SetInt("Block_A_BLOCK_ID", BLOCK_BALL_ID);
    bb.SetString("Block_A_block_type", "RELATIVE");
    bb.SetDouble("Block_A_block_arg", .5);
    bb.SetBool("Block_A_invert_direction", true);
    
    auto token = unique_id::fromRandom();
    tokens.push_back(token);
    
    roboteam_msgs::RoleDirective dir;
    dir.robot_id = own;
    dir.tree = "InterceptorTree";
    dir.blackboard = bb.toMsg();
    dir.token = unique_id::toMsg(token);
    GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher().publish(dir);
}    

void assert_and_erase(std::vector<int>& vec, int i) {
    auto it = std::find(vec.begin(), vec.end(), i);
    assert(it != vec.end());
    vec.erase(it);
}
    
void InterceptorsTactic::Initialize() {
    RTT_DEBUGLN("Initializing");
    roboteam_msgs::World world = LastWorld::get();
    std::vector<int> available = getAvailableRobots();
    
    if (available.size() == 0) {
        RTT_DEBUGLN("No bots available for interceptor tactic");
        return;
    }
    
    std::vector<int> opp_picked;
    
    boost::optional<int> opt;

    auto holder = getBallHolder();
    bool has_holder = (bool) holder;
    if (has_holder) {
        RTT_DEBUGLN("Holder: %d", holder->first.id);
        auto opp = holder->first;
        opt = get_robot_closest_to_point(available, world, opp.pos);
        if (!opt) {
        	failure = true;
        	return;
        }
        assert_and_erase(available, *opt);
        assign(*opt, opp);
        opp_picked.push_back(opp.id);
    } else RTT_DEBUGLN("No Holder");
    
    for (int i = has_holder ? 1 : 0; i < std::min((int) available.size(), 3); i++) {
        
        roboteam_msgs::WorldRobot opp;
        do {
            opp = *get_rand_element_seq<roboteam_msgs::WorldRobot>(world.them);
        } while (sequence_contains(opp_picked, opp.id));

        opt = get_robot_closest_to_point(available, world, opp.pos);
        if (!opt) {
        	failure = true;
        	return;
        }
        RTT_DEBUGLN("Assigning extra bot %d to opponent %d", *opt, opp.id);
        assert_and_erase(available, *opt);
        opp_picked.push_back(opp.id);
        assign(*opt, opp);
    }
}

bt::Node::Status InterceptorsTactic::Update() {
    return failure ? bt::Node::Status::Failure : bt::Node::Status::Running;
}
    
}
