#include "roboteam_tactics/analysis/SpaceAnalysis.h"
#include "roboteam_utils/Clip.h"
#include "boost/polygon/voronoi.hpp"
#include "roboteam_utils/Draw.h"
#include "roboteam_utils/Optimization.h"
#include "roboteam_tactics/utils/utils.h"
#include <boost/geometry/algorithms/correct.hpp> 

#include <vector>
#include <utility>
#include <vector>

namespace rtt {
    
using namespace boost::polygon;
using Vector = Vector2;
using Position = Position;
typedef point_data<double> VPoint;
typedef polygon_traits<Polygon>::point_type PPoint;
typedef voronoi_diagram<double> VoronoiDiagram;
typedef VoronoiDiagram::cell_type Cell;
typedef VoronoiDiagram::edge_type Edge;


void clip_(Vector& v) {
    if (v.x < -4.5) {
        v.x = -4.5;
    }
    if (v.x > 4.5) {
        v.x = 4.5;
    }
    if (v.y < -3) {
        v.y = -3;
    }
    if (v.y > 3) {
        v.y = 3;
    }
}

std::vector<VPoint> extractPoints(const roboteam_msgs::World& world, bool opponentsOnly) {
    std::vector<VPoint> points;
    for (const auto& bot : world.them) {
        points.push_back(VPoint(bot.pos.x * 1000, bot.pos.y * 1000));
    }
    if (!opponentsOnly) {
        for (const auto& bot : world.us) {
            points.push_back(VPoint(bot.pos.x * 1000, bot.pos.y * 1000));
        }
    }
    return points;
}

VoronoiDiagram* buildVD(const std::vector<VPoint>& points) {
    VoronoiDiagram* vd = new VoronoiDiagram();
    construct_voronoi(points.begin(), points.end(), vd);
    return vd;
}

VoronoiDiagram* buildVD(const roboteam_msgs::World& world, bool opponentsOnly) {
    return buildVD(extractPoints(world, opponentsOnly));
}

Polygon clippedPoly(const Cell& cell, const std::vector<VPoint> points) {
    Polygon result;
    std::vector<PPoint> resultPoints;
    const Edge* firstEdge = cell.incident_edge();
    const Edge* currentEdge = firstEdge;
    do {
        PPoint p0, p1;
        if (currentEdge->is_finite()) {
            p0 = construct<PPoint>(currentEdge->vertex0()->x()/1000, currentEdge->vertex0()->y()/1000);
            p1 = construct<PPoint>(currentEdge->vertex1()->x()/1000, currentEdge->vertex1()->y()/1000);
        } else if (currentEdge->vertex0()) {
            auto s1 = points[cell.source_index()];
            auto s2 = points[currentEdge->twin()->cell()->source_index()];
            double x = s1.y() - s2.y();
            double y = -(s1.x() - s2.x());
            double temp = x;
            x += .5 * y;
            y += .5 * temp;
            p0 = construct<PPoint>(currentEdge->vertex0()->x()/1000, currentEdge->vertex0()->y()/1000);
            p1 = construct<PPoint>(x, y);
        } else {
            currentEdge = currentEdge->next();
            continue;
        }
        Vector v0(p0.x(), p0.y());
        Vector v1(p1.x(), p1.y());
        if (clip::cohenSutherlandClip(v0, v1)) {
            p0.x(v0.x);
            p0.y(v0.y);
            p1.x(v1.x);
            p1.y(v1.y);
            resultPoints.push_back(p0);
            resultPoints.push_back(p1);
        }
        currentEdge = currentEdge->next();
    } while (currentEdge != firstEdge);
    result.set(resultPoints.begin(), resultPoints.end());
    return result;
}

Polygon buildPoly(const Cell* cell) {
    std::vector<PPoint> points;
    const Edge* firstEdge = cell->incident_edge();
    const Edge* currentEdge = firstEdge;
    bool finite = true;
    do {
        if (currentEdge->vertex0()) {
            points.push_back(construct<PPoint>(currentEdge->vertex0()->x()/1000, currentEdge->vertex0()->y()/1000));
        }
        if (currentEdge->vertex1()) {
            points.push_back(construct<PPoint>(currentEdge->vertex1()->x()/1000, currentEdge->vertex1()->y()/1000));
        }
        finite &= currentEdge->is_finite();
        currentEdge = currentEdge->next();
    } while (currentEdge != firstEdge);
    
    Polygon poly;
    set_points(poly, points.begin(), points.end());
    boost::polygon::rectangle_data<double> bb;
    boost::polygon::extents(bb, poly);
    poly = bb;
    
    
    return poly;
}

bool edgesAreConnected(const Edge* e1, const Edge* e2) {
    Vector v10(e1->vertex0()->x(), e1->vertex0()->y());
    Vector v11(e1->vertex1()->x(), e1->vertex1()->y());
    Vector v20(e2->vertex0()->x(), e2->vertex0()->y());
    Vector v21(e2->vertex1()->x(), e2->vertex1()->y());
    return v10 == v20 || v10 == v21 || v11 == v20 || v11 == v21;
}

bool cellIsClosed(const Cell& cell) {
    std::vector<const Edge*> edges;
    const Edge* current = cell.incident_edge();
    const Edge* first = current;
    do {
        edges.push_back(current);
        current = current->next();
    } while (current != first);
    
    for (const Edge* e1 : edges) {
        int matches = 0;
        for (const Edge* e2 : edges) {
            if (e1 == e2) continue;
            if (edgesAreConnected(e1, e2)) matches++;
        }
        
        // Every edge should share an endpoint with two others
        if (matches != 2) {
            return false;
        }
    }
    return true;
}

bool cellContains(const Cell& cell, const Polygon& poly, const PPoint& point) {
    if (cellIsClosed(cell)) {
        return contains(poly, point);
    }
    return false;
}

void drawVoronoi(const roboteam_msgs::World& world, bool opponentsOnly) {
    static Draw draw;

    std::vector<VPoint> points;
    for (const auto& bot : world.them) {
        points.push_back(VPoint(bot.pos.x * 1000, bot.pos.y * 1000));
    }
    if (!opponentsOnly) {
        for (const auto& bot : world.us) {
            points.push_back(VPoint(bot.pos.x * 1000, bot.pos.y * 1000));
        }
    }
    
    VoronoiDiagram* vd = buildVD(world, opponentsOnly);
    int count = 0;
    draw.setColor(0, 0, 0);
    for (auto it = vd->edges().begin(); it != vd->edges().end(); it++) {
        char buf[10];
        sprintf(buf, "Voronoi%d", count++);
        Vector v1, v2;
        
        if (it->is_infinite() && it->vertex0()) {
            auto s1 = points[it->cell()->source_index()];
            auto s2 = points[it->twin()->cell()->source_index()];
            double x = (s1.y() - s2.y()) * 1000;
            double y = (s1.x() - s2.x()) * -1000;
            v1 = Vector(it->vertex0()->x() / 1000.0, it->vertex0()->y() / 1000.0);
            v2 = Vector(x, y);
        } else if (it->is_finite()) {
            v1 = Vector(it->vertex0()->x() / 1000.0, it->vertex0()->y() / 1000.0);
            v2 = Vector(it->vertex1()->x() / 1000.0, it->vertex1()->y() / 1000.0);
        } else continue;
        clip_(v1);
        clip_(v2);
        v2 = v2 - v1;
        //clip::CohenSutherlandLineClip(v1, v2);
        draw.drawLine(std::string(buf), v1, v2);
    }
    
    draw.setColor(255, 0, 0);
    for (auto it = vd->vertices().begin(); it != vd->vertices().end(); it++) {
        char buf[10];
        sprintf(buf, "Voronoi%d", count++);
        draw.drawPoint(std::string(buf), Vector(it->x() / 1000.0, it->y() / 1000.0));
    }
}

Territory findTerritory(const roboteam_msgs::WorldRobot& bot, bool our_team, const roboteam_msgs::World& world) {
    VoronoiDiagram* vd = buildVD(world, false);
    for (auto it = vd->cells().begin(); it != vd->cells().end(); it++) {
        Polygon poly = clippedPoly(*it, extractPoints(world, false));
        if (!cellContains(*it, poly, construct<PPoint>(bot.pos.x, bot.pos.y))) continue;
        PPoint c;
        center(c, poly);
        return {{bot.id, our_team}, Position(bot.pos.x, bot.pos.y, bot.angle), poly, Vector(c.x(), c.y()), area(poly)};
    }
    ROS_ERROR("Bot %d not found in findTerritory", bot.id);
    return Territory();
}

std::map<TeamRobot, Territory> findTerritories(const roboteam_msgs::World& world) {
    VoronoiDiagram* vd = buildVD(world, false);
    std::map<TeamRobot, Territory> territories;
    for (auto it = vd->cells().begin(); it != vd->cells().end(); it++) {
        Polygon poly = clippedPoly(*it, extractPoints(world, false));
        Territory tt;
        tt.bot = {9999, false};
        tt.poly = poly;
        tt.area = area(poly);
        PPoint c;
        center(c, poly);
        tt.center = Vector(c.x(), c.y());
        bool found = false;
        for (auto& bot : world.us) {
            PPoint pos = construct<PPoint>(bot.pos.x, bot.pos.y);
            if (cellContains(*it, poly, pos)) {
                tt.bot = {bot.id, true};
                tt.botPos = Position(bot.pos.x, bot.pos.y, bot.angle);
                found = true;
                break;
            }
        }
        for (auto& bot : world.them) {
            PPoint pos = construct<PPoint>(bot.pos.x, bot.pos.y);
            if (true || cellContains(*it, poly, pos)) {
                tt.bot = {bot.id, false};
                tt.botPos = Position(bot.pos.x, bot.pos.y, bot.angle);
                found = true;
                break;
            }
        }
        if (!found) {
            ROS_ERROR("Bot not found in findTerritories!");
        }
        territories[tt.bot] = tt;
    }
    return territories;
}

bool compareByArea(Territory t1, Territory t2) {
    return t1.area > t2.area; // largest territories first
}

std::vector<Territory> territoriesBySize(const roboteam_msgs::World& world, bool opponentsOnly) {
    std::map<TeamRobot, Territory> territories = findTerritories(world);
    std::vector<Territory> ordered;
    for (auto& pair : territories) {
        ordered.push_back(pair.second);
    }
    std::sort(ordered.begin(), ordered.end(), &compareByArea);
    return ordered;
}

double totalDistanceToOpponents(const Vector2& point, const roboteam_msgs::World& world) {
    double total = 0.0;
    for (const auto& bot : world.them) {
        Vector2 botPos(bot.pos);
        total += point.dist2(botPos);
    }
    return sqrtl(total);
}

Vector2 freePositionNear(const Vector2& center, double radius, const roboteam_msgs::World& world) {
    const auto scoreFunc = [world] (const Vector2& vec) {
        return totalDistanceToOpponents(vec, world);
    };
    return sampleForVector(center, -radius, radius, -radius, radius, 100, scoreFunc);
}

Vector2 freePositionWithLOS(const Vector2& center, double radius,
        const Vector2& losTarget, const roboteam_msgs::World& world) {
    const auto scoreFunc = [world, center] (const Vector2& vec) {
        if (!getObstacles(vec, center, &world, true).empty()) return -99999.9;
        return -totalDistanceToOpponents(vec, world);
    };
    return sampleForVector(center, -radius, radius, -radius, radius, 100, scoreFunc);
}

}