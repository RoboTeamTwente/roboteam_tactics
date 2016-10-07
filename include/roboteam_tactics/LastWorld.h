#pragma once

#include "roboteam_msgs/World.h"

class LastWorld {
    public:
    static roboteam_msgs::World get();
    static void set(roboteam_msgs::World world);

    private:
    static roboteam_msgs::World lastWorld;
} ;
