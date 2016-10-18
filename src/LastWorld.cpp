#include "roboteam_msgs/World.h"

#include "roboteam_tactics/LastWorld.h"

namespace rtt {

roboteam_msgs::World LastWorld::lastWorld;
bool LastWorld::blue;

void LastWorld::set(roboteam_msgs::World world) {
    LastWorld::lastWorld = world;
}

roboteam_msgs::World LastWorld::get() {
    return LastWorld::lastWorld;
}

bool LastWorld::we_are_blue() {
    return LastWorld::blue;
}

void LastWorld::set_we_are_blue(bool b) {
    LastWorld::blue = b;
}

}
