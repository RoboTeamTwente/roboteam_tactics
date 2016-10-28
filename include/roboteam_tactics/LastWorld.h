#pragma once

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/GeometryFieldSize.h"

namespace rtt {

class LastWorld {
    public:
    static roboteam_msgs::World get();
    static void set(roboteam_msgs::World world);

    static roboteam_msgs::GeometryFieldSize get_field();
    static void set_field(roboteam_msgs::GeometryFieldSize field);

    static roboteam_msgs::Vector2f PredictBallPos(double t);

    static bool we_are_blue();
    static void set_we_are_blue(bool);

    private:
    static roboteam_msgs::World lastWorld;
    static roboteam_msgs::GeometryFieldSize field;
    static bool blue;
};

}
