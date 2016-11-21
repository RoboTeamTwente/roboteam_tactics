#pragma once

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/GeometryData.h"

namespace rtt {

using namespace roboteam_utils;

class LastWorld {
    public:
    // ros::Subscriber world_sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, msgCallBackGoToPos);
    // ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, msgCallbackFieldGeometry);
// void msgCallBackGoToPos(const roboteam_msgs::WorldConstPtr& world) {
	// rtt::LastWorld::set(*world);
    // may_update = true;
// }

// void msgCallbackFieldGeometry(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    // rtt::LastWorld::set_field(geometry->field);
// }
    static void callback_world_state(const roboteam_msgs::WorldConstPtr& world);
    static void callback_geom_data(const roboteam_msgs::GeometryDataConstPtr& geometry);
    static void register_callbacks(bool world_state = true, bool geometry_data = true);


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
