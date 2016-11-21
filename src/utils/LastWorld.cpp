#include <ros/ros.h>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"

#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

using namespace roboteam_utils;

void LastWorld::callback_world_state(const roboteam_msgs::WorldConstPtr& world) {
    LastWorld::set(*world);
}

void LastWorld::callback_geom_data(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    LastWorld::set_field(geometry->field);
}

void register_callbacks(bool world_state, bool geometry_data) {
    ros::NodeHandle n;

    LastWorld::world_sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, LastWorld::callback_world_state);
    LastWorld::geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, LastWorld::callback_geom_data);
}

roboteam_msgs::World LastWorld::get() {
    return LastWorld::lastWorld;
}

void LastWorld::set(roboteam_msgs::World world) {
    LastWorld::lastWorld = world;
}

roboteam_msgs::GeometryFieldSize LastWorld::get_field() {
    return LastWorld::field;
}

void LastWorld::set_field(roboteam_msgs::GeometryFieldSize field) {
    LastWorld::field = field;
}

roboteam_msgs::Vector2f LastWorld::PredictBallPos(double t) {
	roboteam_msgs::WorldBall ball = lastWorld.ball;
	roboteam_msgs::Vector2f ballVel = ball.vel;
	roboteam_msgs::Vector2f predictedBallPos;
	predictedBallPos.x = ball.pos.x + ballVel.x * t;
	predictedBallPos.y = ball.pos.y + ballVel.y * t;
	return predictedBallPos;
}

Vector2 LastWorld::get_our_goal_center() {
    if (get_our_field_side() == "left") {
        return Vector2(field.field_length / -2, 0);
    } else {
        return Vector2(field.field_length / 2, 0);
    }
}

Vector2 LastWorld::get_their_goal_center() {
    return get_our_goal_center() * -1;
}

ros::Subscriber LastWorld::world_sub;
ros::Subscriber LastWorld::geom_sub;
roboteam_msgs::World LastWorld::lastWorld;
roboteam_msgs::GeometryFieldSize LastWorld::field;

}
