#pragma once

#include "roboteam_tactics/LastWorld.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include <boost/optional.hpp>
#include <vector>
#include <algorithm>

namespace rtt {
    
using Vector = roboteam_utils::Vector2;    
using Position = roboteam_utils::Position;    
    
const std::vector<Vector> GOAL_POINTS_LEFT({
    Vector(-3, .35),
    Vector(-3, 0),
    Vector(-3, -.35)
});
  
const std::vector<Vector> GOAL_POINTS_RIGHT({
    Vector(3, .35),
    Vector(3, 0),
    Vector(3, -.35)
});


std::vector<Vector> our_goal();

bool we_are_left();
bool can_see_our_goal(const roboteam_msgs::WorldRobot& bot);
bool potential_cross_recepient(const roboteam_msgs::WorldRobot& bot);

double distance_score(const roboteam_msgs::WorldRobot& bot);
double orientation_score(const roboteam_msgs::WorldRobot& bot);
double base_danger_score(const roboteam_msgs::WorldRobot& bot);
double danger_score(const roboteam_msgs::WorldRobot& bot);
    
void dump_scores(const roboteam_msgs::World& world);    
    
boost::optional<roboteam_msgs::WorldRobot> charging_bot();
roboteam_msgs::WorldRobot most_dangerous_bot();
roboteam_msgs::WorldRobot second_most_dangerous_bot();
    
}