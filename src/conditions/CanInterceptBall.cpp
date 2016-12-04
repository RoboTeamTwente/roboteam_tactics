#include "roboteam_tactics/conditions/CanInterceptBall.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#ifdef DEBUG_ICPT
#include <cstdio>
#endif

namespace rtt {

RTT_REGISTER_CONDITION(CanInterceptBall);

CanInterceptBall::CanInterceptBall(std::string name, bt::Blackboard::Ptr blackboard) :
    Condition(name, blackboard) { }

bt::Node::Status CanInterceptBall::Update() {
    // TODO: Why is this not implemented? - Bob
    return Status::Failure;
}

struct intercept_data CanInterceptBall::calc_intercept() {
    roboteam_msgs::World world = LastWorld::get();
    roboteam_utils::Vector2 ball_pos(world.ball.pos.x, world.ball.pos.y);
    roboteam_utils::Vector2 ball_vel(world.ball.vel.x, world.ball.vel.y);
    roboteam_msgs::WorldRobot* me = nullptr;
    unsigned int my_id = GetInt("me");
    for (auto& bot : world.us) {
        if (bot.id == my_id) {
            me = &bot;
            break;
        }
    }
    if (me == nullptr) {
        for (auto& bot : world.them) {
            if (bot.id == my_id) {
                me = &bot;
                break;
            }
        }
    }
    if (me == nullptr) {
        ROS_ERROR("(CanInterceptBall::calc_intercept) Could not find bot with id %d!", my_id);
        return {false, ball_pos, -1.0};
    }
    roboteam_utils::Position my_pos(me->pos.x, me->pos.y, me->angle);
    roboteam_utils::Position my_vel(me->vel.x, me->vel.y, me->w);
    return CanInterceptBall::calc_intercept(my_pos, my_vel, ball_pos, ball_vel);
}

}
#ifdef DEBUG_ICPT

int main(int argc, char** argv) {
    roboteam_utils::Position bot_pos(3.0, 3.0, 0.0);
    roboteam_utils::Position bot_vel(0.7, -.3, 0.0);
    roboteam_utils::Vector2 ball_pos(6.0, 6.0);
    roboteam_utils::Vector2 ball_vel(-.25, .25);
    struct intercept_data icpt = rtt::CanInterceptBall::calc_intercept(bot_pos, bot_vel, ball_pos, ball_vel);
    printf("Success: %d\nIcpt pos: (%f, %f)\nIcpt angle: %f\nIcpt time: %f\n", icpt.success, icpt.icpt_pos.x, icpt.icpt_pos.y,
        (icpt.icpt_pos - bot_pos.location()).angle(), icpt.time);
}

#endif
