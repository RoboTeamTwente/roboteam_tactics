//
// Created by emiel on 2-5-18.
//

#include <tuple>
#include <vector>
#include <cmath>

#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/RobotPatternGenerator.h"

namespace rtt {

	/**
	 * Creates a line on which robots are placed. Relative to a centre. Returns coordinates in radians
	 *
	 * @param numRobots - The number of robots that should be in the pattern.
	 * @param lineWidth - The total width of the line in meters
	 * @param centre    - The center of the pattern, e.g. our goal / the ball / a dangerous robot
	 * @param angleToCentre - 0 rad -> to the left of the centre; PI rad -> to the right of the centre
	 * @param distanceFromCentre - The perpendicular distance between the line and the centre
	 * @return A tuple with two float vectors, giving positions in radians relative to the centre.
	 * 		   The first contains angles towards the centre, the second contains distances from the centre.
	 */
	std::tuple<std::vector<float>, std::vector<float>> RobotPatternGenerator::Line(int numRobots, float lineWidth, Vector2 centre, float angleToCentre, float distanceFromCentre){

		std::vector<float> angles;
		std::vector<float> distances;

		// Calculate the step between each robot. In this case, distance in meters
		float step = lineWidth / (numRobots - 1);
		// Calculate the offset of the 1st robot
		float offset = -lineWidth / 2;

		// For each robot
		for (int i = 0; i < numRobots; i++) {
			// Calculate the y-position
			float y = offset + i * step;
			// Calculate the long edge of the triangle
			float x = sqrt(pow(distanceFromCentre, 2) + pow(y, 2));
			// Calculate the corner
			float w = atan(y / distanceFromCentre);
			// Add the angleToCentre
			w += angleToCentre;

			angles.push_back(w);
			distances.push_back(x);

		}

		return std::make_tuple(angles, distances);
	}

	std::tuple<std::vector<float>, std::vector<float>> RobotPatternGenerator::Circle(int numRobots, float circleAngle, float circleRadius, Vector2 centre, float angleToCentre, float distanceFromCentre) {

		std::vector<float> angles;
		std::vector<float> distances;

		distanceFromCentre -= circleRadius;

		// Calculate the step between each robot. In this case, distance in meters
		float step = circleAngle / (numRobots - 1);
		// Calculate the position of the 1st robot
		float offset = -circleAngle / 2;

		// For each robot
		for (int i = 0; i < numRobots; i++) {

			float A = offset + i * step;
			A += angleToCentre;

			float b = circleRadius + distanceFromCentre;
			float c = circleRadius;

			float b2 = pow(b, 2);
			float c2 = pow(c, 2);

			float a2 = b2 + c2 - 2 * b * c * cos(A);
			float a = sqrt(a2);

			float C = acos( (a2 + b2 - c2) / ( 2 * a * b ));

			if((A-angleToCentre) < 0)
				C = -C;

//			ROS_WARN_STREAM_NAMED("plays.JimKOD", i << ": "
//			<< " A=" << A
//			<< " a=" << a << " a2=" << a2
//			<< " b=" << b << " b2=" << b2
//          << " c=" << c << " c2=" << c2
//			<< " C=" << C);

			angles.push_back(C);
			distances.push_back(a);

		}



		return std::make_tuple(angles, distances);
	}


}