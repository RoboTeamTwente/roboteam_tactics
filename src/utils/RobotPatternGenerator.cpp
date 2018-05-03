//
// Created by emiel on 2-5-18.
//

#include <vector>
#include <cmath>
#include <ros/ros.h>

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

	std::vector<Vector2> RobotPatternGenerator::Line(int numRobots, float lineWidth, Vector2 centre, float angleToCentre, float distanceFromCentre){

		std::vector<Vector2> x_y_coords;

		// Calculate the step between each robot. In this case, distance in meters
		float step = lineWidth / (numRobots - 1);
		// Calculate the offset of the 1st robot
		float offset = -lineWidth / 2;

		// For each robot
		for (int i = 0; i < numRobots; i++) {
			// Calculate the y-position
			float x = distanceFromCentre;
			float y = offset + i * step;
			// Rotate according to angleToCentre
			float _x = x * cos(angleToCentre)  + y * sin(angleToCentre);
			float _y = -x * sin(angleToCentre) + y * cos(angleToCentre);
			// Create Vector2
			Vector2 coords(_x, _y);
			// Add centre
			coords = coords + centre;
			// Store coordinates
			x_y_coords.push_back(coords);
		}

		return x_y_coords;
	}

	std::vector<Vector2> RobotPatternGenerator::Circle(int numRobots, float circleAngle, float circleRadius, Vector2 centre, float phase, float angleToCentre, float distanceFromCentre){

		std::vector<Vector2> x_y_coords;

		// Calculate the step between each robot. In this case, distance in meters
		float step = circleAngle / (numRobots - 1);
		// Calculate the offset of the 1st robot
		float offset = -circleAngle / 2;

		// For each robot
		for (int i = 0; i < numRobots; i++) {

			/* Calculate coordinates in circle */
			// Calculate angle
			float angle = offset + i * step;
			// Add phase
			angle += phase;
			// Convert to cartesian coordinates
			float x = circleRadius * cos(angle);
			float y = circleRadius * sin(angle);

			// Add distanceFromCentre
			x += distanceFromCentre;
			// Rotate according to angleToCentre
			float _x = x * cos(angleToCentre)  + y * sin(angleToCentre);
			float _y = -x * sin(angleToCentre) + y * cos(angleToCentre);

			// Create Vector2
			Vector2 coords(_x, _y);
			// Add centre
			coords = coords + centre;
			// Store coordinates
			x_y_coords.push_back(coords);
		}

		return x_y_coords;
	}

}