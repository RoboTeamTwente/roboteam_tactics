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
	 * Creates a line on which robots are placed. Relative to a centre. Returns cartesian coordinates
	 *
	 * @param numRobots - The number of robots that should be in the pattern.
	 * @param lineWidth - The total width of the line in meters
	 * @param centre    - The centre of the pattern, e.g. our goal / the ball / a dangerous robot
	 * @param angleToCentre - 0 rad -> to the left of the centre; PI/2 rad -> above the centre;
	 * @param distanceFromCentre - The perpendicular distance between the line and the centre
	 * @return A vector of Vector2, with each Vector2 containing a position of a robot
	 */
	std::vector<Vector2> RobotPatternGenerator::Line(int numRobots, float lineWidth, Vector2 centre, float angleToCentre, float distanceFromCentre){

		std::vector<Vector2> x_y_coords;

		if(numRobots == 0)
			return x_y_coords;

		// Calculate the offset of the 1st robot
		float offset = -lineWidth / 2;

		// Calculate the step between each robot. In this case, distance in meters
		float step = 0;
		if(numRobots > 1)
			step = lineWidth / (numRobots - 1);


		// For each robot
		for (int i = 0; i < numRobots; i++) {
			// Calculate the y-position
			float x = distanceFromCentre;
			float y = offset + i * step;
			std::cout<<"Old - x: "<<x<<" - y: "<<y<<std::endl;
			// Rotate according to angleToCentre
			float _x = x * cos(angleToCentre)  + y * sin(angleToCentre);
			float _y = -x * sin(angleToCentre) + y * cos(angleToCentre);
			std::cout<<"New - x: "<<_x<<" - y: "<<_y<<std::endl;
			// Create Vector2
			Vector2 coords(_x, _y);
			// Add centre
			coords = coords + centre;
			std::cout<<coords<<std::endl;
			// Store coordinates
			x_y_coords.push_back(coords);
		}

		return x_y_coords;
	}

	/**
	 * Creates a circle on which robots are placed. Relative to a centre. Returns cartesian coordinates
	 *
	 * @param numRobots - The number of robots that should be in the pattern.
	 * @param circleAngle - The total angle that the pattern should cover. M_PI -> half of a circle; M_PI/2 -> quarter of a circle;
	 * @param circleRadius - The radius of the circle
	 * @param centre    - The centre of the pattern, e.g. our goal / the ball / a dangerous robot
	 * @param phase - The phase of the circle. Sets which degrees the pattern should cover. circleRadius=M_PI & phase=0.1*M_PI -> covers 0.1*M_PI to 1.1*M_PI
	 * @param angleToCentre - 0 rad -> to the left of the centre; PI/2 rad -> above the centre
	 * @param distanceFromCentre - The perpendicular distance between the centre of the circle and the centre
	 * @return A vector of Vector2, with each Vector2 containing a position of a robot
	 */
	std::vector<Vector2> RobotPatternGenerator::Circle(int numRobots, float circleAngle, float circleRadius, Vector2 centre, float phase, float angleToCentre, float distanceFromCentre){

		std::vector<Vector2> x_y_coords;

		// Calculate the step between each robot. In this case, distance in meters
		float step = 0;
		if(numRobots > 1)
			step = circleAngle / (numRobots - 1);
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