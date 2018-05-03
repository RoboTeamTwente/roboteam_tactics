//
// Created by emiel on 2-5-18.
//
#pragma once

#include "roboteam_utils/Vector2.h"
#include <vector>

namespace rtt{

	class RobotPatternGenerator {
	public:
		static std::vector<Vector2> Line(int numRobots, float lineWidth, Vector2 centre, float angleToCentre, float distanceFromCentre);
		static std::vector<Vector2> Circle(int numRobots, float circleAngle, float circleRadius, Vector2 centre, float phase, float angleToCentre, float distanceFromCentre);

	};


}