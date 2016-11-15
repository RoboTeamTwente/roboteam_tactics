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
		angle = 2 * asin((0.5*radius) / (center-start).length());
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

roboteam_utils::Vector2 Cone::ClosestPointOnSideTwoCones(Cone otherCone, roboteam_utils::Vector2 point) {
	if (!(this->IsWithinCone(point) && otherCone.IsWithinCone(point))) {
		ROS_WARN("This point is not inside either of the cones");
		return point;
	}

	// Check whether's there's an easy way out
	roboteam_utils::Vector2 closestSideCone1 = this->ClosestPointOnSide(point);
	if (!otherCone.IsWithinCone(closestSideCone1)) {
		return closestSideCone1; 
	}

	roboteam_utils::Vector2 closestSideCone2 = otherCone.ClosestPointOnSide(point);
	if (!this->IsWithinCone(closestSideCone2)) {
		return closestSideCone2; 
	}
	ROS_INFO_STREAM("going to intersectoin");

	// Where do the two cones cross?
	roboteam_utils::Vector2 cone1Side1 = (center-start).rotate(angle);
	roboteam_utils::Vector2 cone1Side2 = (center-start).rotate(-angle);
	roboteam_utils::Vector2 cone2Side1 = (otherCone.center-otherCone.start).rotate(otherCone.angle);
	roboteam_utils::Vector2 cone2Side2 = (otherCone.center-otherCone.start).rotate(-otherCone.angle);

	ROS_INFO_STREAM("intersection1, start1: " << start.x << " " << start.y << " dir1: " << cone1Side1.x << " " << cone1Side1.y << " start2: " << otherCone.start.x << " " << otherCone.start.y << " dir2: " << cone2Side1.x << " " << cone2Side1.y);
	ROS_INFO_STREAM("intersection2, start1: " << start.x << " " << start.y << " dir1: " << cone1Side1.x << " " << cone1Side1.y << " start2: " << otherCone.start.x << " " << otherCone.start.y << " dir2: " << cone2Side2.x << " " << cone2Side2.y);
	ROS_INFO_STREAM("intersection3, start1: " << start.x << " " << start.y << " dir1: " << cone1Side2.x << " " << cone1Side2.y << " start2: " << otherCone.start.x << " " << otherCone.start.y << " dir2: " << cone2Side1.x << " " << cone2Side1.y);
	ROS_INFO_STREAM("intersection4, start1: " << start.x << " " << start.y << " dir1: " << cone1Side2.x << " " << cone1Side2.y << " start2: " << otherCone.start.x << " " << otherCone.start.y << " dir2: " << cone2Side2.x << " " << cone2Side2.y);

	roboteam_utils::Vector2 intersection1 = LineIntersection(start, cone1Side1, otherCone.start, cone2Side1);
	roboteam_utils::Vector2 intersection2 = LineIntersection(start, cone1Side1, otherCone.start, cone2Side2);
	roboteam_utils::Vector2 intersection3 = LineIntersection(start, cone1Side2, otherCone.start, cone2Side1);
	roboteam_utils::Vector2 intersection4 = LineIntersection(start, cone1Side2, otherCone.start, cone2Side2);

	ROS_INFO_STREAM("intersection1: " << intersection1.x << " " << intersection1.y);
	ROS_INFO_STREAM("intersection2: " << intersection2.x << " " << intersection2.y);
	ROS_INFO_STREAM("intersection3: " << intersection3.x << " " << intersection3.y);
	ROS_INFO_STREAM("intersection4: " << intersection4.x << " " << intersection4.y);

	roboteam_utils::Vector2 closestIntersection = intersection1;
	if ((intersection2 - point).length() < (closestIntersection - point).length()) closestIntersection = intersection2;
	if ((intersection3 - point).length() < (closestIntersection - point).length()) closestIntersection = intersection3;
	if ((intersection4 - point).length() < (closestIntersection - point).length()) closestIntersection = intersection4;
	return closestIntersection;
}

roboteam_utils::Vector2 Cone::LineIntersection(roboteam_utils::Vector2 line1Start, roboteam_utils::Vector2 line1Dir, roboteam_utils::Vector2 line2Start, roboteam_utils::Vector2 line2Dir) {
	float slope1 = line1Dir.y / line1Dir.x;
	float slope2 = line2Dir.y / line2Dir.x;
	float intersectX = (slope1*line1Start.x - slope2*line2Start.x - line1Start.y + line2Start.y) / (slope1 - slope2);
	float intersectY = slope1 * (intersectX - line1Start.x) + line1Start.y;
	return roboteam_utils::Vector2(intersectX, intersectY);
}

bool Cone::DoConesOverlap(Cone otherCone) {
	roboteam_utils::Vector2 cone1Side1 = (center-start).rotate(angle);
	roboteam_utils::Vector2 cone1Side2 = (center-start).rotate(-angle);
	roboteam_utils::Vector2 cone2Side1 = (otherCone.center-otherCone.start).rotate(otherCone.angle);
	roboteam_utils::Vector2 cone2Side2 = (otherCone.center-otherCone.start).rotate(-otherCone.angle);
	double angleDiff1 = cone1Side1.angle() - cone2Side2.angle();
	double angleDiff2 = cone1Side2.angle() - cone2Side1.angle();
	if (angleDiff1 > 0 && angleDiff2 > 0) {
		return false;
	} else if (angleDiff1 < 0 && angleDiff2 < 0) {
		return false;
	} else if ((fabs(angleDiff1) < 2*angle && fabs(angleDiff1 < 2*otherCone.angle)) || (fabs(angleDiff2) < 2*angle && fabs(angleDiff2 < 2*otherCone.angle))) {
		return true;
	} else {
		return false;
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

	if (angle > otherCone.angle) {
		if (cone2Side2.angle() > cone1Side2.angle() && cone2Side1.angle() < cone1Side1.angle()) {
			// ROS_INFO_STREAM("one cone inside another");
			return *this;
		}
	} else {
		if (cone1Side2.angle() > cone2Side2.angle() && cone1Side1.angle() < cone2Side1.angle()) {
			// ROS_INFO_STREAM("one cone inside another");
			return otherCone;
		}
	}

	roboteam_utils::Vector2 newCenter;
	double newRadius;
	if (fabs(angleDiff1) > fabs(angleDiff2)) {
		newCenter = cone2Side2.rotate(0.5*angleDiff1) + start;
		newRadius = (cone2Side2 - (newCenter-start)).length();
	} else {
		newCenter = cone2Side1.rotate(0.5*angleDiff2) + start;
		newRadius = (cone2Side1 - (newCenter-start)).length();
	}

	Cone newCone(start, newCenter, newRadius);

	// Debug info:
	// cone1Side1 = cone1Side1.scale(1/cone1Side1.length());
	// cone1Side2 = cone1Side2.scale(1/cone1Side2.length());
	// cone2Side1 = cone2Side1.scale(1/cone2Side1.length());
	// cone2Side2 = cone2Side2.scale(1/cone2Side2.length());
	// roboteam_utils::Vector2 cone3Side1 = (newCone.center-newCone.start).rotate(newCone.angle);
	// cone3Side1 = cone3Side1.scale(1/cone3Side1.length());
	// roboteam_utils::Vector2 cone3Side2 = (newCone.center-newCone.start).rotate(-newCone.angle);
	// cone3Side2 = cone3Side2.scale(1/cone3Side2.length());
	// cone3Side1 = cone3Side1.scale(1/cone3Side1.length());
	// cone3Side2 = cone3Side2.scale(1/cone3Side2.length());

	// ROS_INFO_STREAM("cone1Side1:" << cone1Side1.x << " " << cone1Side1.y);
	// ROS_INFO_STREAM("cone1Side2:" << cone1Side2.x << " " << cone1Side2.y);
	// ROS_INFO_STREAM("cone2Side1:" << cone2Side1.x << " " << cone2Side1.y);
	// ROS_INFO_STREAM("cone2Side2:" << cone2Side2.x << " " << cone2Side2.y);
	// ROS_INFO_STREAM("cone3Side1:" << cone3Side1.x << " " << cone3Side1.y);
	// ROS_INFO_STREAM("cone3Side2:" << cone3Side2.x << " " << cone3Side2.y);

	return newCone;
}

Cone::~Cone(){}

} // rtt