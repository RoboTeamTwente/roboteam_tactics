#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/LastWorld.h"

namespace rtt {

TeamHasBall::TeamHasBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<TeamHasBall>(name);
    blue_team = GetBool("blue_team");
}

bt::Node::Status TeamHasBall::Update() {
    const auto bots = blue_team ? LastWorld::get().robots_blue : LastWorld::get().robots_yellow;
    bt::Blackboard tmp(*blackboard); //copy
    bt::Blackboard::Ptr tmp_ptr = std::make_shared<bt::Blackboard>(tmp);
    std::string tmp_name = "tmp_ihb";
    for (const auto& bot : bots) {
        tmp.SetInt("me", bot.id);
        IHaveBall ihb(tmp_name, tmp_ptr);
        if (ihb.Update() == Status::Failure)
            return Status::Failure;
    }
    return Status::Success;
}

}