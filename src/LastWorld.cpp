#include "roboteam_msgs/World.h"

#include "roboteam_tactics/LastWorld.h"

roboteam_msgs::World LastWorld::lastWorld;

void LastWorld::set(roboteam_msgs::World world) {
    LastWorld::lastWorld = world;
}

roboteam_msgs::World LastWorld::get() {
    return LastWorld::lastWorld;
}

