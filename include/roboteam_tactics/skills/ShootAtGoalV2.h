#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Cone.h"
#include "roboteam_msgs/World.h"
#include <vector>
#include "boost/optional.hpp"

namespace rtt {

struct Section {
    Vector2 a, b, center;
    double length;
    Section(double ax, double ay, double bx, double by) :
        a(ax, ay), b(bx, by), center((ax+bx)/2.0, (ay+by)/2.0),
        length(a.dist(b)) {}
    Section(Vector2 a, Vector2 b) : Section(a.x, a.y, b.x, b.y) {}
    Section() {}
    
    Vector2 intersection(const Section& other) const;
    bool pointOnLine(const Vector2& point) const;
};
    
class GoalPartition {
public:
    static const Section LEFT_GOAL;
    static const Section RIGHT_GOAL;
    GoalPartition(bool leftSide);
    
    void calculatePartition(const roboteam_msgs::World& world, const Vector2& shooterPos);
    boost::optional<Section> largestOpenSection() const;
    std::vector<Section> getBlocked() const;
    std::vector<Section> getOpen() const;
    std::vector<Section> getRobots() const;
    void reset();
private:
    const Section goalSection;
    std::vector<Section> blocked, open, robots;
};

class ShootAtGoalV2 : public Skill {
public:
    ShootAtGoalV2(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    
private:
    static constexpr double ACCEPTABLE_DEVIATION = .05; // rad
    static constexpr double KICK_VEL = 7.5; // .5 m/s safety margin
    GoalPartition partition;
};

}