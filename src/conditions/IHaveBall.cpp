#include <cmath>
#include <cstdio>
#include "ros/ros.h"

#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(IHaveBall);

IHaveBall::IHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    
}

boost::optional<roboteam_msgs::WorldRobot> IHaveBall::find_bot_pos(const roboteam_msgs::World& world) const {
    for (const auto& bot : (us ? world.us : world.them)) {
        if (bot.id == (unsigned int) me)
            return boost::optional<roboteam_msgs::WorldRobot>(bot);
    }
    return boost::optional<roboteam_msgs::WorldRobot>();
}

bt::Node::Status IHaveBall::Update() {
    me = GetInt("me", blackboard->GetInt("ROBOT_ID"));
    us = GetBool("our_team");
    roboteam_msgs::World world = LastWorld::get();
    auto opt_bot = getWorldBot(me, us);
    if (!opt_bot) {
        return Status::Failure;
    }
    bool has = bot_has_ball(*opt_bot, world.ball);
    auto res = has ? Status::Success : Status::Failure;

    return res;
}

std::vector<roboteam_msgs::World> IHaveBall::success_states() const {
    roboteam_msgs::World succ1, succ2;
    succ1.us.push_back(roboteam_msgs::WorldRobot());
    succ1.us[0].angle = M_PI_2l; //90 deg
    succ1.ball.pos.y = 0.1;

    succ2.us.push_back(roboteam_msgs::WorldRobot());
    succ2.us[0].pos.x = 42;
    succ2.us[0].pos.y = 33;
    succ2.ball.pos.x = 42.1;
    succ2.ball.pos.y= 33;

    std::vector<roboteam_msgs::World> vec;
    vec.push_back(succ1);
    vec.push_back(succ2);
    return vec;
}

std::vector<roboteam_msgs::World> IHaveBall::fail_states() const {
    roboteam_msgs::World succ1, succ2;
    succ1.us.push_back(roboteam_msgs::WorldRobot());
    succ1.us[0].angle = M_PI_2l; //90 deg
    succ1.ball.pos.y = 0.4; // too far

    succ2.us.push_back(roboteam_msgs::WorldRobot());
    succ2.us[0].pos.x = 42;
    succ2.us[0].pos.y = 33;
    succ2.ball.pos.x = 42;
    succ2.ball.pos.y= 33.1; // wrong orientation

    std::vector<roboteam_msgs::World> vec;
    vec.push_back(succ1);
    vec.push_back(succ2);
    return vec;
}

}
