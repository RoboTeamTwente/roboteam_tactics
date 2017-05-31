#pragma once

#include <vector>
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Section.h"
#include "roboteam_msgs/World.h"

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
    GoalPartition();

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

    struct BlockedSectionSorter {
        bool operator()(const Section& a, const Section& b) {
            return a.a.y > b.a.y;
        }
    };
private:
    const Section goalSection;
    std::vector<Section> blocked, open, robots;

    static Section calcSection();
};

}


