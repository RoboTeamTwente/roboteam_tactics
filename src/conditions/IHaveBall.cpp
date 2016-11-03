#include <cmath>
#include <cstdio>
#include "ros/ros.h"

#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/LastWorld.h"

namespace rtt {

IHaveBall::IHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<IHaveBall>(name);
    me = GetInt("me");
    us = GetBool("our_team");
}

boost::optional<roboteam_msgs::WorldRobot> IHaveBall::find_bot_pos(const roboteam_msgs::World& world) const {
    for (const auto& bot : (us ? world.us : world.them)) {
        if (bot.id == (unsigned int) me)
            return boost::optional<roboteam_msgs::WorldRobot>(bot);
    }
    return boost::optional<roboteam_msgs::WorldRobot>();
}

bt::Node::Status IHaveBall::Update() {
    roboteam_msgs::World world = LastWorld::get();
    roboteam_msgs::Vector2f ball = world.ball.pos;

    boost::optional<roboteam_msgs::WorldRobot> bot = find_bot_pos(world);

    if (!bot) {
        return Status::Invalid;
    }

    roboteam_utils::Vector2 ball_vec(ball.x, ball.y), bot_vec(bot->pos.x, bot->pos.y);
    roboteam_utils::Vector2 ball_norm = (ball_vec - bot_vec);

    double dist = ball_norm.length();
    double angle = ball_norm.angle();

    if (dist > 0.14 || fabs(angle - bot->angle) > 0.4) {
        return Status::Failure;
    }

    return Status::Success;
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
