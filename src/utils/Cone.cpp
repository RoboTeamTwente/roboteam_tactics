#include "ros/ros.h"
#include "roboteam_tactics/utils/Cone.h"
#include "roboteam_utils/Vector2.h"

#include <cmath>

namespace rtt {

Cone::Cone(roboteam_utils::Vector2 startPoint, roboteam_utils::Vector2 centerPoint, double distance) {
	start = startPoint;
	center = centerPoint;
	radius = distance;
	MakeCone();
}

void Cone::MakeCone() {
	double angle;
	if (radius > 0) {
		angle = 2.0 * asin((center-start).length() / (0.5*radius));
	} else {
		ROS_WARN("radius for cone invalid!");
		return;
	}
	firstSide = (center-start).rotate(-angle);
	secondSide = (center-start).rotate(angle);
	return;
}

bool Cone::IsWithinCone(roboteam_utils::Vector2 point) {
	return true;
}

roboteam_utils::Vector2 Cone::ClosestPointOnSide(roboteam_utils::Vector2) {
	return roboteam_utils::Vector2(0.0, 0.0);
}

Cone::~Cone(){} // NOP

} // rtt