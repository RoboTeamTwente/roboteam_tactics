#pragma once

#include <boost/optional.hpp>
#include <string>
#include <vector>

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/LastWorld.h"

namespace rtt {

std::vector<std::string> getNodesSubscribedTo(std::string topic);
std::string getMyNamespace();
boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>> getBallHolder();
std::vector<roboteam_msgs::WorldRobot> getObstacles(const roboteam_msgs::WorldRobot& bot,
                                                    const roboteam_utils::Vector2& point,
                                                    const roboteam_msgs::World* world_ptr = nullptr,
                                                    bool sight_only = false);;
} // rtt
