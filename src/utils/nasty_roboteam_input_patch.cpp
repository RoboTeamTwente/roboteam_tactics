#include <boost/optional.hpp>
#include <string>

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/World.h"

#include "roboteam_tactics/bt.hpp"

std::string bt::Node::status_desc = "";

namespace rtt {
    bool bot_has_ball(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::WorldBall& ball) {
        return false;
    }

    boost::optional<roboteam_msgs::WorldRobot> lookup_bot(unsigned int id, bool our_team, const roboteam_msgs::World* world) {
        return boost::none;
    }
}
