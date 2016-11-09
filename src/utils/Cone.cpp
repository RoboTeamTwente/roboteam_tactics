#include "ros/ros.h"
#include "roboteam_tactics/utils/Cone.h"
#include "roboteam_utils/Vector2.h"

#include <cmath>

namespace rtt {

Cone::Cone(roboteam_utils::Vector2 startPoint, roboteam_utils::Vector2 centerPoint, double distance) {
	start = startPoint;
	center = centerPoint;
	radius = distance;
	if (radius > 0) {
		angle = 2.0 * asin((0.5*radius) / (center-start).length());
	} else {
		ROS_WARN("radius for cone invalid!");
	}
}

double Cone::CleanAngle(double cleanangle) {
	if (cleanangle <= M_PI && cleanangle >= -M_PI) {
		return cleanangle;
	} else if (cleanangle > M_PI) {
		cleanangle -= 2*M_PI;
		return CleanAngle(cleanangle);
	} else if (cleanangle < M_PI) {
		cleanangle += 2*M_PI;
		return CleanAngle(cleanangle);
	}
	return 0.0;
}

bool Cone::IsWithinCone(roboteam_utils::Vector2 point) {
	roboteam_utils::Vector2 vectorToPoint = point-start;
	roboteam_utils::Vector2 vectorToCenter = center-start;
	if (fabs(CleanAngle(vectorToPoint.angle()-vectorToCenter.angle())) < angle) {
		return true;
	} else {
		return false;
	}
}

roboteam_utils::Vector2 Cone::ClosestPointOnSide(roboteam_utils::Vector2 point) {
	if (!IsWithinCone(point)) {
		ROS_WARN("This point is not inside the cone");
		return roboteam_utils::Vector2(0.0, 0.0);
	}
	roboteam_utils::Vector2 vectorToPoint = point-start;
	roboteam_utils::Vector2 vectorToCenter = center-start;
	double pointAngle = CleanAngle((vectorToPoint-vectorToCenter).angle());
	roboteam_utils::Vector2 closestSide;
	if (pointAngle >= 0) {
		closestSide = vectorToCenter.rotate(angle).scale(vectorToPoint.length() / vectorToCenter.length());
	} else {
		closestSide = vectorToCenter.rotate(-angle).scale(vectorToPoint.length() / vectorToCenter.length());
	}
	return closestSide;
}

Cone::~Cone(){}

} // rtt