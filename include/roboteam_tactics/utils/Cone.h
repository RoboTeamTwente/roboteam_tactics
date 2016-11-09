#pragma once

#include "roboteam_utils/Vector2.h"

namespace rtt
{

class Cone {
public:
	Cone(roboteam_utils::Vector2 startPoint, roboteam_utils::Vector2 centerPoint, double distance);
	bool IsWithinCone(roboteam_utils::Vector2 point);
	roboteam_utils::Vector2 ClosestPointOnSide(roboteam_utils::Vector2);
	~Cone();
private:
	roboteam_utils::Vector2 start;
	roboteam_utils::Vector2 center;
	double radius;
	double angle;
};

} // rtt