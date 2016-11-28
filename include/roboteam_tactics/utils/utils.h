#pragma once

#include <boost/optional.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <iostream>

#include "roboteam_tactics/bt.hpp"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/LastWorld.h"

namespace rtt {

template<typename T>
void delete_from_vector(std::vector<T> &items, const T &item) {
    auto it = std::find(items.begin(), items.end(), item);
    if (it != items.end()) {
        items.erase(it);
    }
}

std::vector<std::string> getNodesSubscribedTo(std::string topic);
std::string getMyNamespace();
boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>> getBallHolder();
std::vector<roboteam_msgs::WorldRobot> getObstacles(const roboteam_msgs::WorldRobot& bot,
                                                    const roboteam_utils::Vector2& point,
                                                    const roboteam_msgs::World* world_ptr = nullptr,
                                                    bool sight_only = false);

/**
 * Predict the ball or robot position in the future based on the current velocity
 */
roboteam_utils::Vector2 predictBallPos(double seconds);
roboteam_utils::Vector2 predictRobotPos(uint robot_id, bool our_team, double seconds);

/**
 * Looks up the given bot on the given team in the given world, and returns an optional WorldRobot.
 * If you don't pass a pointer to world, the function will get a world through LastWorld.
 */
boost::optional<roboteam_msgs::WorldRobot> lookup_bot(unsigned int id, bool our_team, const roboteam_msgs::World* world = nullptr);
boost::optional<roboteam_msgs::WorldRobot> lookup_our_bot(unsigned int id, const roboteam_msgs::World* world = nullptr);
boost::optional<roboteam_msgs::WorldRobot> lookup_their_bot(unsigned int id, const roboteam_msgs::World* world = nullptr);

bool bot_has_ball(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::WorldBall& ball);

static bool bot_has_ball(unsigned int id, bool our_team, const roboteam_msgs::WorldBall& ball) { return bot_has_ball(*lookup_bot(id, our_team), ball); }

void print_blackboard(const bt::Blackboard::Ptr bb, std::ostream& out = std::cout);
void merge_blackboards(bt::Blackboard::Ptr target, const bt::Blackboard::Ptr extras);

/**
 * Gets a random integer in the range [0, max).
 */
int get_rand_int(int max);

/**
 * Gets a random integer in the range [min, max).
 */
int get_rand_int(int min, int max);

/**
 * Gets a random real number in the range [0, max).
 */
float get_rand_real(float max);

/**
 * Gets a random real number in the range [min, max).
 */
float get_rand_real(float min, float max);

using time_point = std::chrono::steady_clock::time_point;
using seconds = std::chrono::seconds;
using milliseconds = std::chrono::milliseconds;

time_point now();

seconds time_difference_seconds(time_point start, time_point end);
milliseconds time_difference_milliseconds(time_point start, time_point end);

std::string describe_status(bt::Node::Status status);

std::string get_our_field_side();
roboteam_msgs::RobotCommand stop_command(unsigned int id);

bool is_digits(const std::string &str);

int get_robot_closest_to_their_goal(std::vector<int> robots, const roboteam_msgs::World &world = LastWorld::get());
int get_robot_closest_to_ball(std::vector<int> robots, const roboteam_msgs::World &world = LastWorld::get());

} // rtt
