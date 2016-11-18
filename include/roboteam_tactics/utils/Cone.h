#pragma once

#include "roboteam_utils/Vector2.h"

namespace rtt {

class Cone {
public:
	Cone(roboteam_utils::Vector2 startPoint, roboteam_utils::Vector2 centerPoint, double distance);
	double CleanAngle(double angle);
	bool IsWithinCone(roboteam_utils::Vector2 point);
	bool IsWithinField(roboteam_utils::Vector2 point);
	roboteam_utils::Vector2 ClosestPointOnSide(roboteam_utils::Vector2);
	roboteam_utils::Vector2 ClosestPointOnSideTwoCones(Cone otherCone, roboteam_utils::Vector2 point);
	static roboteam_utils::Vector2 LineIntersection(roboteam_utils::Vector2 line1Start, roboteam_utils::Vector2 line1Dir, roboteam_utils::Vector2 line2Start, roboteam_utils::Vector2 line2Dir);
	bool DoConesOverlap(Cone otherCone);
	Cone MergeCones(Cone otherCone);
	~Cone();

	roboteam_utils::Vector2 start;
	roboteam_utils::Vector2 center;
	double radius;
	double angle;
private:
};

} // rtt