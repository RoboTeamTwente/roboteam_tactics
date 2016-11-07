#pragma once

#include "roboteam_utils/Vector2.h"

namespace rtt
{

class Cone {
public:
	Cone(roboteam_utils::Vector2 startPoint, roboteam_utils::Vector2 centerPoint, double radius);
	~Cone();
private:
	roboteam_utils::Vector2 start;
	roboteam_utils::Vector2 center;
	double radius;
	roboteam_utils::Vector2 firstSide;
	roboteam_utils::Vector2 secondSide;
};

} // rtt