#include "ros/ros.h"

#include <boost/range/join.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/filtered.hpp>

#include "roboteam_msgs/World.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {
    
RTT_REGISTER_CONDITION(CanSeePoint);

CanSeePoint::CanSeePoint(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<CanSeePoint>(name);
    threshold_dist = GetBool("check_move") ? .35 : .15;
}

bt::Node::Status CanSeePoint::Update() {
    roboteam_msgs::WorldRobot* me = nullptr;
    roboteam_msgs::World world = LastWorld::get();
    unsigned int id = GetInt("ROBOT_ID");
    auto all_bots = boost::join(world.us, world.them);
    for (auto& bot : all_bots) {
        if (bot.id == id) {
            me = &bot;
            break;
        }
    }
    if (!me) {
        ROS_WARN("CanSeePoint::Update was unable to find a bot with id %d.", id);
        return bt::Node::Status::Invalid;
    }
    Vector2 target = Vector2(GetDouble("x_coor"), GetDouble("y_coor"));
    if (getObstacles(*me, target, &world, GetBool("check_move")).empty()) {
        return bt::Node::Status::Success;
    }
    return bt::Node::Status::Failure;
    /* 
    Vector2 my_pos = Vector2(me->pos.x, me->pos.y);
    
    auto positions = all_bots | 
        boost::adaptors::filtered([id](roboteam_msgs::WorldRobot& bot) {
            return bot.id != id;
        }) |
        boost::adaptors::transformed([](roboteam_msgs::WorldRobot& bot) {
            return Vector2(bot.pos.x, bot.pos.y); 
        });
    
    for (const Vector2& pos : positions) {
        Vector2 projection = pos.project(my_pos, target);
        double proj_dist = projection.dist(pos);
        double dist_to_start = my_pos.dist(pos);
        if (proj_dist < threshold_dist && dist_to_start > .0001) {
            return bt::Node::Status::Failure;
        }
    }
    return bt::Node::Status::Success;*/
}
    
}
