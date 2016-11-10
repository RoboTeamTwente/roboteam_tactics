#pragma once

#include <boost/optional.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <iostream>

#include "roboteam_tactics/bt/Blackboard.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
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
                                                    bool sight_only = false);
boost::optional<roboteam_msgs::WorldRobot> lookup_bot(unsigned int id, bool our_team, const roboteam_msgs::World* world = nullptr);
bool bot_has_ball(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::WorldBall& ball);
static bool bot_has_ball(unsigned int id, bool our_team, const roboteam_msgs::WorldBall& ball) { return bot_has_ball(*lookup_bot(id, our_team), ball); }

void print_blackboard(const bt::Blackboard::Ptr bb, std::ostream& out = std::cout);
void merge_blackboards(bt::Blackboard::Ptr target, const bt::Blackboard::Ptr extras);

/**
 * Gets a random number in the range [0, max).
 */
int get_rand(int max);

/**
 * Gets a random number in the range [min, max).
 */
int get_rand(int min, int max);

using time_point = std::chrono::steady_clock::time_point;
using seconds = std::chrono::seconds;

time_point now();

seconds time_difference(time_point start, time_point end);

} // rtt
