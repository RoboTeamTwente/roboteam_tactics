#include "roboteam_tactics/tactics/HaltTactic.h"

#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"

namespace rtt {
    
HaltTactic::HaltTactic(std::string name, bt::Blackboard::Ptr bb) : Tactic(name, bb) {}
    
void HaltTactic::Initialize() {
    static auto pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    
    // Unclaim all robots
    RobotDealer::halt_override();
    
    auto world = LastWorld::get();
    for (const auto& bot : world.us) {
        RobotDealer::claim_robot(bot.id);

        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", bot.id);
        bb.SetInt("repeat", -1); // Freeze forever

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = bot.id;
        rd.tree = "HaltRole";
        rd.blackboard = bb.toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        rd.token = unique_id::toMsg(token);

        pub.publish(rd);

        tokens.push_back(token);
    } 
}

bt::Node::Status HaltTactic::Update() {
    return LastRef::getState() == RefState::HALT ? bt::Node::Status::Running : bt::Node::Status::Success;
}

}
