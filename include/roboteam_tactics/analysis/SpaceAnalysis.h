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
    Position botPos;
    Polygon poly;
    Vector2 center;
    double area;
} Territory;
   
std::map<TeamRobot, Territory> findTerritories(const roboteam_msgs::World& world = LastWorld::get());

std::vector<Territory> territoriesBySize(const roboteam_msgs::World& world = LastWorld::get(), bool opponentsOnly = true);
    
void drawVoronoi(const roboteam_msgs::World& world = LastWorld::get(), bool opponentsOnly = true);

double totalDistanceToOpponents(const Vector2& point, const roboteam_msgs::World& world = LastWorld::get());

Vector2 freePositionNear(const Vector2& center, double radius, const roboteam_msgs::World& world = LastWorld::get());
    
}