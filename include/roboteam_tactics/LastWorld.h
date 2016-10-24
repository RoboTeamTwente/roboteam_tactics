#pragma once

#include "roboteam_msgs/World.h"

namespace rtt {

class LastWorld {
    public:
    static roboteam_msgs::World get();
    static void set(roboteam_msgs::World world);
    
    static bool we_are_blue();
    static void set_we_are_blue(bool);

    private:
    static roboteam_msgs::World lastWorld;
    static bool blue;
};

}
