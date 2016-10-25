#pragma once

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"

namespace rtt {

class LastWorld {
    public:
    static roboteam_msgs::World get();
    static void set(roboteam_msgs::World world);
    static roboteam_msgs::Vector2f PredictBallPos(double t);

    private:
    static roboteam_msgs::World lastWorld;
} ;

}