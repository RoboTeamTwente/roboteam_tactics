//
// Created by emiel on 2-5-18.
//
#pragma once

namespace rtt{

	class RobotPatternGenerator {
	public:
		static std::tuple<std::vector<float>, std::vector<float>> Line(int numRobots, float lineWidth, Vector2 centre, float angleToCentre, float distanceFromCentre);
		static std::tuple<std::vector<float>, std::vector<float>> Circle(int numRobots, float circleAngle, float circleRadius, Vector2 centre, float angleToCentre, float distanceFromCentre);

	};


}