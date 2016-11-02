#pragma once

#include <string>
#include <vector>
#include <boost/optional.hpp>
#include <utility>
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

std::vector<std::string> getNodesSubscribedTo(std::string topic);
boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>> getBallHolder();

} // rtt

