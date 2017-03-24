#include "roboteam_tactics/skills/ShootAtGoalV2.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/skills/Kick.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include <iostream>

namespace rtt {
    
RTT_REGISTER_SKILL(ShootAtGoalV2);
    
const Section GoalPartition::LEFT_GOAL = Section(-4.5, 0.5, -4.5, -0.5);
const Section GoalPartition::RIGHT_GOAL = Section(4.5, 0.5, 4.5, -0.5);
    
Vector2 Section::intersection(const Section& other) const {
    double x1 = a.x,
           x2 = b.x,
           y1 = a.y,
           y2 = b.y,
           
           x3 = other.a.x,
           x4 = other.b.x,
           y3 = other.a.y,
           y4 = other.b.y;
           
    //https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    double divisor = (x1-x2) * (y3-y4) - (y1 - y2) * (x3 - x4);
    double t1 = (x1 * y2 - y1 * x2);
    double t2 = (x3 * y4 - y3 * x4);
    double x = (t1 * (x3 - x4) - (x1 - x2) * t2) / divisor;
    double y = (t1 * (y3 - y4) - (y1 - y2) * t2) / divisor;
    return Vector2(x, y);
}    
    
bool Section::pointOnLine(const Vector2& point) const {
    return point.dist(a) + point.dist(b) - length < .0001;
}    
    
GoalPartition::GoalPartition(bool leftSide) : goalSection(leftSide ? LEFT_GOAL : RIGHT_GOAL) {}

struct BlockedSectionSorter {
    bool operator()(const Section& a, const Section& b) {
        return a.a.y > b.a.y;
    }
};

void GoalPartition::calculatePartition(const roboteam_msgs::World& world, const Vector2& shooterPos) {
    Cone cone(shooterPos, goalSection.center, .5);
    std::vector<roboteam_msgs::WorldRobot> bots = getRobotsInCone(world, cone);
    
    robots.clear();
    
    for (const auto& bot : bots) {
        Vector2 pos(bot.pos);
        if (pos == shooterPos) continue;
        Vector2 botToShooter = shooterPos - pos;
        Vector2 perp(botToShooter.y, -botToShooter.x);
        Section botSect(
            perp.stretchToLength( .09) + pos,
            perp.stretchToLength(-.09) + pos
        );
        robots.push_back(botSect);
    }

    for (const Section& sect : robots) {
        Vector2 lineA = sect.a - shooterPos, lineB = sect.b - shooterPos;
        Section sectA(shooterPos, lineA.stretchToLength(9.0) + shooterPos);
        Section sectB(shooterPos, lineB.stretchToLength(9.0) + shooterPos);
        Vector2 isectA = sectA.intersection(goalSection);
        Vector2 isectB = sectB.intersection(goalSection);
        std::cout << "isectA:" << isectA.x << ", " << isectA.y << "\n";
        std::cout << "isectB:" << isectB.x << ", " << isectB.y << "\n";
        
        if (goalSection.pointOnLine(isectA) && goalSection.pointOnLine(isectB)) {
            if (isectA.y >= isectB.y) {
                blocked.emplace_back(isectA, isectB);
            } else {
                blocked.emplace_back(isectB, isectA);
            }
        } else if (goalSection.pointOnLine(isectA)) {
            blocked.emplace_back(isectA, goalSection.b);
        } else if (goalSection.pointOnLine(isectB)) {
            blocked.emplace_back(goalSection.a, isectB);
        }
    }
    
    std::sort<std::vector<Section>::iterator, BlockedSectionSorter>(blocked.begin(), blocked.end(), BlockedSectionSorter());
    // Sections with greater Y are now at the beginning of the vector,
    // and every Section's a.y value is greater than its b.y value.
        
    Vector2 openStart = goalSection.a;
    for(size_t i = 0; openStart.y >= goalSection.b.y && i < blocked.size(); i++) {
        Section blockedSection = blocked.at(i);
        open.emplace_back(openStart, blockedSection.a);
        openStart = blockedSection.b;
    }
    if (openStart != goalSection.b) {
        open.emplace_back(openStart, goalSection.b);
    } 
}

boost::optional<Section> GoalPartition::largestOpenSection() const {
    if (open.empty()) return boost::none;
    
    Section maxSection = open.at(0);
    double  maxLength = maxSection.length;
    for (size_t i = 1; i < open.size(); i++) {
        Section section = open.at(i);
        double len = section.length;
        if (len > maxLength) {
            maxLength = len;
            maxSection = section;
        }
    }
    return maxSection;
}
    
std::vector<Section> GoalPartition::getBlocked() const {
    return blocked;
}    
    
std::vector<Section> GoalPartition::getOpen() const {
    return open;
}    
    
std::vector<Section> GoalPartition::getRobots() const {
    return robots;
}    

void GoalPartition::reset() {
    blocked.clear();
    open.clear();
    robots.clear();
}

ShootAtGoalV2::ShootAtGoalV2(std::string name, bt::Blackboard::Ptr blackboard) 
        : Skill(name, blackboard), partition(get_our_side() == "left") {

}

bt::Node::Status ShootAtGoalV2::Update() {
    const roboteam_msgs::World world = LastWorld::get();
    auto bot = getWorldBot(blackboard->GetInt("ROBOT_ID"));
    const Vector2 ownPos(bot->pos);
    const double orientation = bot->angle;
    
    partition.reset();
    partition.calculatePartition(world, ownPos);
    
    auto largest = partition.largestOpenSection();
    if (!largest) {
        return bt::Node::Status::Invalid;
    }
    
    Vector2 target = largest->center;
    double targetAngle = (target - ownPos).angle();
    
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    if (fabs(targetAngle - orientation) < ACCEPTABLE_DEVIATION) {
        ScopedBB(*bb, "SAGV2_kick")
            .setInt("ROBOT_ID", bot->id)
            .setBool("wait_for_signal", false)
            .setDouble("kickVel", KICK_VEL);
        Kick kick("SAGV2_kick", bb);
        return kick.Update();
    } else {
        ScopedBB(*bb, "SAGV2_rotate")
            .setInt("ROBOT_ID", bot->id)
            .setString("center", "ball")
            .setDouble("faceTowardsPosx", target.x)
            .setDouble("faceTowardsPosy", target.y)
            .setDouble("w", /* summon control engineer here */ 2.0);
        RotateAroundPoint rap("SAGV2_rotate", bb);
        return rap.Update();
    }
}
    
}