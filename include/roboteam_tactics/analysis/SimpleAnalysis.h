#pragma once

#include <ros/message_forward.h>

namespace roboteam_msgs {

ROS_DECLARE_MESSAGE(World);

} // roboteam_msgs

namespace rtt {

bool countOurRobots(roboteam_msgs::World const & world);

} // namespace rtt
