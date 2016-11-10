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
		return point;
	}
	roboteam_utils::Vector2 vectorToPoint = point-start;
	roboteam_utils::Vector2 vectorToCenter = center-start;
	double pointAngle = CleanAngle(vectorToPoint.angle()-vectorToCenter.angle());
	roboteam_utils::Vector2 closestSide;
	if (pointAngle >= 0) {
		closestSide = vectorToCenter.rotate(angle).scale(vectorToPoint.length() / vectorToCenter.length()) + start;
	} else {
		closestSide = vectorToCenter.rotate(-angle).scale(vectorToPoint.length() / vectorToCenter.length()) + start;
	}
	return closestSide;
}

bool Cone::DoConesOverlap(Cone otherCone) {
	roboteam_utils::Vector2 cone1Side1 = (center-start).rotate(angle);
	roboteam_utils::Vector2 cone1Side2 = (center-start).rotate(-angle);
	roboteam_utils::Vector2 cone2Side1 = (otherCone.center-otherCone.start).rotate(otherCone.angle);
	roboteam_utils::Vector2 cone2Side2 = (otherCone.center-otherCone.start).rotate(-otherCone.angle);
	double angleDiff1 = cone1Side1.angle() - cone2Side2.angle();
	double angleDiff2 = cone1Side2.angle() - cone2Side1.angle();
	ROS_INFO_STREAM("angleDiff1: " << angleDiff1 << " angleDiff2: " << angleDiff2);
	if (angleDiff1 > 0 && angleDiff2 > 0) {
		return false;
	} else if (angleDiff1 < 0 && angleDiff2 < 0) {
		return false;
	// } else if ((fabs(angleDiff1) > fabs(angle) && fabs(angleDiff1) > fabs(otherCone.angle)) || (fabs(angleDiff2) > fabs(angle) && fabs(angleDiff2) > fabs(otherCone.angle))) {
		// return false;
	} else {
		return true;
	}
}

Cone Cone::MergeCones(Cone otherCone) {
	if (!DoConesOverlap(otherCone)) {
		ROS_WARN("No overlap");
		return *this;
	}

	roboteam_utils::Vector2 cone1Side1 = (center-start).rotate(angle);
	roboteam_utils::Vector2 cone1Side2 = (center-start).rotate(-angle);
	roboteam_utils::Vector2 cone2Side1 = (otherCone.center-otherCone.start).rotate(otherCone.angle);
	roboteam_utils::Vector2 cone2Side2 = (otherCone.center-otherCone.start).rotate(-otherCone.angle);
	double angleDiff1 = cone1Side1.angle() - cone2Side2.angle();
	double angleDiff2 = cone1Side2.angle() - cone2Side1.angle();

	roboteam_utils::Vector2 newCenter;
	double newRadius;
	if (fabs(angleDiff1) > fabs(angleDiff2)) {
		newCenter = cone2Side2.rotate(0.5*angleDiff1) + start;
		newRadius = (otherCone.center - newCenter).length();
	} else {
		newCenter = cone2Side1.rotate(0.5*angleDiff2) + start;
		newRadius = (center - newCenter).length();
	}

	Cone newCone(start, newCenter, newRadius);
	return newCone;
}

Cone::~Cone(){}

} // rtt