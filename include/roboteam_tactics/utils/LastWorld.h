#pragma once
#include <ros/ros.h>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/GeometryData.h"

namespace rtt {

using namespace roboteam_utils;

class LastWorld {
    public:
    static void callback_world_state(const roboteam_msgs::WorldConstPtr& world);
    static void callback_geom_data(const roboteam_msgs::GeometryDataConstPtr& geometry);

    static roboteam_msgs::World get();
    static void set(roboteam_msgs::World world);

    static roboteam_msgs::GeometryFieldSize get_field();
    static void set_field(roboteam_msgs::GeometryFieldSize field);

    static roboteam_msgs::Vector2f PredictBallPos(double t);

    static Vector2 get_our_goal_center();
    static Vector2 get_their_goal_center();

    static ros::Subscriber world_sub;
    static ros::Subscriber geom_sub;

    private:
    static roboteam_msgs::World lastWorld;
    static roboteam_msgs::GeometryFieldSize field;

};

}

// Can be placed in a main() to set up the wrld and geom callbacks
// DON'T MAKE SUBSCRIBERS PUBLISHER STATIC/GLOBAL! You'll get a weird
// exception that's hard to debug (probably).

#define RTT_CREATE_WORLD_AND_GEOM_CALLBACKS \
    auto RTT__WORLD__SUBSCRIBER =  n.subscribe<roboteam_msgs::World> ("world_state", 1, rtt::LastWorld::callback_world_state); \
    auto RTT_GEOM_SUBSCRIBER = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1, rtt::LastWorld::callback_geom_data);
