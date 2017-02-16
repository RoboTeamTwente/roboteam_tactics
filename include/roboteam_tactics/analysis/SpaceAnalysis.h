#pragma once

#include "roboteam_utils/LastWorld.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/TeamRobot.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "boost/polygon/polygon.hpp"
#include <map>

namespace rtt {
    
typedef boost::polygon::polygon_data<double> Polygon;
    
typedef struct Territory {
    TeamRobot bot;
    roboteam_utils::Position botPos;
    Polygon poly;
    roboteam_utils::Vector2 center;
    double area;
} Territory;
   
std::map<TeamRobot, Territory> findTerritories(const roboteam_msgs::World& world = LastWorld::get());

std::vector<Territory> territoriesBySize(const roboteam_msgs::World& world = LastWorld::get(), bool opponentsOnly = true);
    
void drawVoronoi(const roboteam_msgs::World& world = LastWorld::get(), bool opponentsOnly = true);

double totalDistanceToOpponents(const roboteam_utils::Vector2& point, const roboteam_msgs::World& world = LastWorld::get());

roboteam_utils::Vector2 freePositionNear(const roboteam_utils::Vector2& center, double radius, const roboteam_msgs::World& world = LastWorld::get());
    
}