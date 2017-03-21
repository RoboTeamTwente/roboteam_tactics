#pragma once

#include "roboteam_utils/LastWorld.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/TeamRobot.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "boost/polygon/polygon.hpp"
#include <map>

namespace rtt {

/**
 * \typedef Polygon
 * \brief The polygon type used in SpaceAnalysis.
 */    
typedef boost::polygon::polygon_data<double> Polygon;
    
/**
 * \struct Territory
 * \brief Information on a robots 'territory'; the area of the field where it has the most influence.
 */
typedef struct Territory {
    TeamRobot bot;   /**< The referenced robot */ 
    Position botPos; /**< The position of the robot (in standard coordinates) */
    Polygon poly;    /**< The polygon describing the outer edges of the territory */
    Vector2 center;  /**< The center of poly */
    double area;     /**< The total area covered by this territory, in m^2 */
} Territory;
   
/**
 * \brief Finds territories for all robots. Every part of the field will be covered by exactly one robot's territory.
 */   
std::map<TeamRobot, Territory> findTerritories(const roboteam_msgs::World& world = LastWorld::get());

/**
 * \brief Gets all territories, sorted by total surface area, smallest first.
 */
std::vector<Territory> territoriesBySize(const roboteam_msgs::World& world = LastWorld::get(), bool opponentsOnly = true);
   
/**
 * \brief Draws a voronoi diagram of the territories to RQT.
 */ 
void drawVoronoi(const roboteam_msgs::World& world = LastWorld::get(), bool opponentsOnly = true);

/**
 * \brief Gets the sum of distances to all robots from a given point.
 */
double totalDistanceToOpponents(const Vector2& point, const roboteam_msgs::World& world = LastWorld::get());

/**
 * \brief Finds an unobstructed location near a desired one.
 * \param center The 'ideal' location, near which we want to be.
 * \param radius The maximum distance we want to be away from the center
 * \param world The world to search in.
 * \return The position within the circle defined by center and radius which has the highest cumulative distance to opponents.
 */
Vector2 freePositionNear(const Vector2& center, double radius, const roboteam_msgs::World& world = LastWorld::get());
    
}