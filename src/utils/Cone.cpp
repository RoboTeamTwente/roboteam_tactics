#include "ros/ros.h"
#include "roboteam_tactics/utils/Cone.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/GeometryFieldSize.h"


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
	side1 = (center-start).rotate(angle);
	side2 = (center-start).rotate(-angle);
}

Cone::Cone(roboteam_utils::Vector2 startPoint, roboteam_utils::Vector2 side1, roboteam_utils::Vector2 side2) {
	this->start = startPoint;
	this->side1 = side1.scale(2 / side1.length());
	this->side2 = side2.scale(2 / side2.length());
	this->angle = CleanAngle(this->side1.angle() - this->side2.angle());
	this->center = this->side2.rotate(0.5*angle) + this->start;
	this->radius = (this->center - this->side1).length();
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
	// ROS_INFO_STREAM("vectorToPoint: " << vectorToPoint.x << " " << vectorToPoint.y);
	// ROS_INFO_STREAM("vectorToCenter: " << vectorToCenter.x << " " << vectorToCenter.y);
	// ROS_INFO_STREAM("vectorToPoint angle " << vectorToPoint.angle() << " vectorToCenter angle " << vectorToCenter.angle() << " angle " << angle);
	if (fabs(CleanAngle(vectorToPoint.angle()-vectorToCenter.angle())) < angle) {
		return true;
	} else {
		return false;
	}
}

bool Cone::IsWithinField(roboteam_utils::Vector2 point) {
 	roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
 	double fieldLimitX = field.field_length / 2.0;
	double fieldLimitY = field.field_width / 2.0;
	if (point.x > fieldLimitX || point.x < -fieldLimitX || point.y > fieldLimitY || point.y < -fieldLimitY) {
		return false;
	} else {
		return true;
	}
}

roboteam_utils::Vector2 Cone::ClosestPointOnSide(roboteam_utils::Vector2 point, roboteam_utils::Vector2 closeTo) {
	if (!IsWithinCone(point)) {
		ROS_WARN("This point is not inside the cone");
		return point;
	}

	roboteam_utils::Vector2 option1 = side1.scale(10 / side1.length()).closestPointOnVector(start, point);
	roboteam_utils::Vector2 option2 = side2.scale(10 / side2.length()).closestPointOnVector(start, point);

	double option1Dist = (closeTo-option1).length();
	double option2Dist = (closeTo-option2).length();

	if (option1Dist <= option2Dist) {
		if (IsWithinField(option1)) return option1;
		else if (IsWithinField(option2)) return option2;
		else return option1;
	} else {
		if (IsWithinField(option2)) return option2;
		else if (IsWithinField(option1)) return option1;
		else return option2;
	}
}

roboteam_utils::Vector2 Cone::SecondClosestPointOnSide(roboteam_utils::Vector2 point) {
	if (!IsWithinCone(point)) {
		ROS_WARN("This point is not inside the cone");
		return point;
	}
	roboteam_utils::Vector2 vectorToPoint = point-start;
	roboteam_utils::Vector2 vectorToCenter = center-start;
	double pointAngle = CleanAngle(vectorToPoint.angle()-vectorToCenter.angle());
	roboteam_utils::Vector2 option1 = vectorToCenter.rotate(angle).scale(vectorToPoint.length() / vectorToCenter.length()) + start;
	roboteam_utils::Vector2 option2 = vectorToCenter.rotate(-angle).scale(vectorToPoint.length() / vectorToCenter.length()) + start;

	

	if (pointAngle >= 0) {
		if (IsWithinField(option2)) return option2;
		else if (IsWithinField(option1)) return option1;
	} else {
		if (IsWithinField(option1)) return option1;
		else if (IsWithinField(option2)) return option2;
	}
	return point;
}

roboteam_utils::Vector2 Cone::ClosestPointOnSideTwoCones(Cone otherCone, roboteam_utils::Vector2 point, roboteam_utils::Vector2 closeTo) {
	if (!(this->IsWithinCone(point) && otherCone.IsWithinCone(point))) {
		ROS_WARN("This point is not inside either of the cones");
		return point;
	}

	roboteam_utils::Vector2 closestPointSoFar(100.0, 100.0);

	roboteam_utils::Vector2 option1 = side1.scale(10 / side1.length()).closestPointOnVector(start, point);
	roboteam_utils::Vector2 option2 = side2.scale(10 / side2.length()).closestPointOnVector(start, point);
	roboteam_utils::Vector2 option3 = otherCone.side1.scale(10 / otherCone.side1.length()).closestPointOnVector(otherCone.start, point);
	roboteam_utils::Vector2 option4 = otherCone.side2.scale(10 / otherCone.side2.length()).closestPointOnVector(otherCone.start, point);

	// Find the one of these options that is closest to closeTo
	if (!otherCone.IsWithinCone(option1) && (option1 - closeTo).length() - (closestPointSoFar - closeTo).length()) closestPointSoFar = option1;
	if (!otherCone.IsWithinCone(option2) && (option2 - closeTo).length() - (closestPointSoFar - closeTo).length()) closestPointSoFar = option2;
	if (!this->IsWithinCone(option3) && (option3 - closeTo).length() - (closestPointSoFar - closeTo).length()) closestPointSoFar = option3;
	if (!this->IsWithinCone(option4) && (option4 - closeTo).length() - (closestPointSoFar - closeTo).length()) closestPointSoFar = option4;

	// If one of these is found, return immediately because they will always be the closest
	if ((closestPointSoFar - closeTo).length() < 100.0) {
		return closestPointSoFar;
	}

	// Otherwise, find the closest intersection of the two cones
	roboteam_utils::Vector2 intersection1 = LineIntersection(start, side1, otherCone.start, otherCone.side1);
	roboteam_utils::Vector2 intersection2 = LineIntersection(start, side1, otherCone.start, otherCone.side2);
	roboteam_utils::Vector2 intersection3 = LineIntersection(start, side2, otherCone.start, otherCone.side1);
	roboteam_utils::Vector2 intersection4 = LineIntersection(start, side2, otherCone.start, otherCone.side2);
	
	if ((intersection1 - closeTo).length() < (closestPointSoFar - closeTo).length() && IsWithinField(intersection1)) closestPointSoFar = intersection1;
	if ((intersection2 - closeTo).length() < (closestPointSoFar - closeTo).length() && IsWithinField(intersection2)) closestPointSoFar = intersection2;
	if ((intersection3 - closeTo).length() < (closestPointSoFar - closeTo).length() && IsWithinField(intersection3)) closestPointSoFar = intersection3;
	if ((intersection4 - closeTo).length() < (closestPointSoFar - closeTo).length() && IsWithinField(intersection4)) closestPointSoFar = intersection4;
	if (closestPointSoFar.length() > 100.0) {closestPointSoFar = point;}
	return closestPointSoFar;
}

roboteam_utils::Vector2 Cone::LineIntersection(roboteam_utils::Vector2 line1Start, roboteam_utils::Vector2 line1Dir, roboteam_utils::Vector2 line2Start, roboteam_utils::Vector2 line2Dir) {
	float slope1 = line1Dir.y / line1Dir.x;
	float slope2 = line2Dir.y / line2Dir.x;
	float intersectX = (slope1*line1Start.x - slope2*line2Start.x - line1Start.y + line2Start.y) / (slope1 - slope2);
	float intersectY = slope1 * (intersectX - line1Start.x) + line1Start.y;
	return roboteam_utils::Vector2(intersectX, intersectY);
}

bool Cone::DoConesOverlap(Cone otherCone) {
	double angleDiff1 = side1.angle() - otherCone.side2.angle();
	double angleDiff2 = side2.angle() - otherCone.side1.angle();
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

	double angleDiff1 = side1.angle() - otherCone.side2.angle();
	double angleDiff2 = side2.angle() - otherCone.side1.angle();

	if (angle > otherCone.angle) {
		if (otherCone.side2.angle() > side2.angle() && otherCone.side1.angle() < side1.angle()) {
			return *this;
		}
	} else {
		if (side2.angle() > otherCone.side2.angle() && side1.angle() < otherCone.side1.angle()) {
			return otherCone;
		}
	}

	roboteam_utils::Vector2 newCenter;
	double newRadius;
	if (fabs(angleDiff1) > fabs(angleDiff2)) {
		newCenter = otherCone.side2.rotate(0.5*angleDiff1) + start;
		newRadius = (otherCone.side2 - (newCenter-start)).length();
	} else {
		newCenter = otherCone.side1.rotate(0.5*angleDiff2) + start;
		newRadius = (otherCone.side1 - (newCenter-start)).length();
	}

	Cone newCone(start, newCenter, newRadius);
	return newCone;
}

Cone::~Cone(){}

} // rtt