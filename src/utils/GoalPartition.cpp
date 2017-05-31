#include "roboteam_tactics/utils/GoalPartition.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/Cone.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_msgs/GeometryFieldSize.h"

namespace rtt {

GoalPartition::GoalPartition() : goalSection(calcSection()) {}

Section GoalPartition::calcSection() {
	auto geom = LastWorld::get_field();
	double x = geom.field_width;
	double halfY = geom.goal_width / 2;
	return Section(x, halfY, x, -halfY);
}

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


void GoalPartition::draw() const {
	static Draw d;
	static unsigned int lineCounter = 0;

	while (lineCounter > 0) {
		d.removeLine("SAGV2-Line" + std::to_string(lineCounter--));
	}

	d.setColor(127, 127, 127);
	for (const Section& sec : robots) {
		d.drawLineAbs("SAGV2-Line" + std::to_string(++lineCounter), sec.a, sec.b);
	}

	d.setColor(255, 0, 0);
	for (const Section& sec : blocked) {
		d.drawLineAbs("SAGV2-Line" + std::to_string(++lineCounter), sec.a, sec.b);
	}

	d.setColor(0, 255, 0);
	for (const Section& sec : open) {
		d.drawLineAbs("SAGV2-Line" + std::to_string(++lineCounter), sec.a, sec.b);
	}
}

}
