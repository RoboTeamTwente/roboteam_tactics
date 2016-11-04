#pragma once

#include <boost/optional.hpp>
#include <string>
#include <vector>

#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

std::vector<std::string> getNodesSubscribedTo(std::string topic);
std::string getMyNamespace();
boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>> getBallHolder();

} // rtt
