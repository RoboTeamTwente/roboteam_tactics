#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(TeamHasBall);

TeamHasBall::TeamHasBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<TeamHasBall>(name);
    our_team = GetBool("our_team");
}

bt::Node::Status TeamHasBall::Update() {
    const auto bots = our_team ? LastWorld::get().us : LastWorld::get().them;
    bt::Blackboard tmp(*blackboard); //copy
    bt::Blackboard::Ptr tmp_ptr = std::make_shared<bt::Blackboard>(tmp);
    std::string tmp_name = "tmp_ihb";
    for (const auto& bot : bots) {
        tmp_ptr->SetInt("ROBOT_ID", bot.id);
        tmp_ptr->SetBool("our_team", our_team);
        IHaveBall ihb(tmp_name, tmp_ptr);
        if (ihb.Update() == Status::Success) {
            return Status::Success;
        }
    }
    return Status::Failure;
}

}
