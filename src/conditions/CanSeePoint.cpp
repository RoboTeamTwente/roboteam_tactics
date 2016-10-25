#include "roboteam_tactics/conditions/CanSeePoint.h"
#include <boost/range/join.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"

namespace rtt {
    
CanSeePoint::CanSeePoint(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    assert_valid<CanSeePoint>(name);
}

bt::Node::Status CanSeePoint::Update() {
    roboteam_msgs::WorldRobot* me = nullptr;
    roboteam_msgs::World world = LastWorld::get();
    unsigned int id = GetInt("me");
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
    
    roboteam_utils::Vector2 my_pos = roboteam_utils::Vector2(me->pos.x, me->pos.y);
    roboteam_utils::Vector2 target = roboteam_utils::Vector2(GetDouble("x_coor"), GetDouble("y_coor"));
    
    auto positions = all_bots | 
        boost::adaptors::filtered([id](roboteam_msgs::WorldRobot& bot) {
            return bot.id != id;
        }) |
        boost::adaptors::transformed([](roboteam_msgs::WorldRobot& bot) {
            return roboteam_utils::Vector2(bot.pos.x, bot.pos.y); 
        });
    
    for (const roboteam_utils::Vector2& pos : positions) {
        roboteam_utils::Vector2 projection = pos.project(my_pos, target);
        double proj_dist = projection.dist(pos);
        double dist_to_start = my_pos.dist(pos);
        if (proj_dist < .4 && dist_to_start > .0001) {
            return bt::Node::Status::Failure;
        }
    }
    return bt::Node::Status::Success;
}
    
}