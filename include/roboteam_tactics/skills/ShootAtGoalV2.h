#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Section.h"
#include "roboteam_utils/Cone.h"
#include "roboteam_msgs/World.h"
#include <vector>
#include "boost/optional.hpp"

namespace rtt {

/**
 * \class GoalPartition
 * \brief Partitions the goal line into a vector of open and closed Sections,
 * from the perspective of a particular point.
 * Open sections are those which can be seen without obstruction from any robots
 * from that point, closed sections are the opposite case.
 */
class GoalPartition {
public:
    GoalPartition(bool leftSide);
    
    /**
     * \brief (Re-)calculates the partition.
     * \param world The world to search for obstacles in.
     * \param shooterPos The position to look from.
     */
    void calculatePartition(const roboteam_msgs::World& world, const Vector2& shooterPos);
    
    /**
     * \brief Gets the largest open section, if one exists.
     * \return If the entire goal line is blocked, an empty optional. Otherwise,
     * an optional containing the largest open section.
     */
    boost::optional<Section> largestOpenSection() const;
    
    /**
     * \return All closed sections.
     */
    std::vector<Section> getBlocked() const;
    
    /**
     * \return All open sections.
     */
    std::vector<Section> getOpen() const;
    
    /**
     * \return The line segments used internally to represent robots.
     */
    std::vector<Section> getRobots() const;
    
    /**
     * \brief Resets all data.
     */
    void reset();

    void draw() const;
private:
    const Section goalSection;
    std::vector<Section> blocked, open, robots;

    static Section calcSection(bool leftSide);
};

/**
 * \class ShootAtGoalV2
 * \brief See YAML
 */
/*
 * Descr: |
 *     Improved ShootAtGoal which finds the ideal place to aim, then shoots the ball there.
 *     This skill will calculate the largest area of the goal which the active robot can
 *     see unobstructed, and aim for the middle of that area.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The robot which is to shoot
 */
class ShootAtGoalV2 : public Skill {
public:
    ShootAtGoalV2(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    
private:
    static constexpr double ACCEPTABLE_DEVIATION = .02; // rad
    static constexpr double KICK_VEL = 7.5; // .5 m/s safety margin
    GoalPartition partition;
};

}
