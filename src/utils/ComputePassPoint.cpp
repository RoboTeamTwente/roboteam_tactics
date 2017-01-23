#include "roboteam_tactics/utils/ComputePassPoint.h"

#include "roboteam_tactics/utils/Math.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/utils/debug_print.h"

#include <iostream>
#include <fstream>

#define RTT_CURRENT_DEBUG_TAG ComputePassPoint

namespace rtt {

PassPoint::PassPoint() {
	std::fstream myfile("src/roboteam_tactics/src/utils/passpoint_weights.txt", std::ios_base::in);
	myfile >> distToGoalWeight >> distToOppWeight >> distToBallWeight >> viewOfGoalWeight >> distOppToBallTrajWeight;

	RTT_DEBUG("Initializing PassPoint and loading weights \n");
	// RTT_DEBUG("distToGoalWeight: %f \n", distToGoalWeight);
	// RTT_DEBUG("distToOppWeight: %f \n", distToOppWeight);
	// RTT_DEBUG("distToBallWeight: %f \n", distToBallWeight);
	// RTT_DEBUG("viewOfGoalWeight: %f \n", viewOfGoalWeight);
	// RTT_DEBUG("distOppToBallTrajWeight: %f \n", distOppToBallTrajWeight);
}

// Calculates the distance between the closest opponent and the testPosition
double PassPoint::calcDistToClosestOpp(roboteam_utils::Vector2 testPosition, roboteam_msgs::World world) {
	double shortestDistance = (Vector2(world.them.at(0).pos) - testPosition).length();
	for (size_t i = 1; i < world.them.size(); i++) {
		double testDistance = (Vector2(world.them.at(i).pos) - testPosition).length();
		if (testDistance < shortestDistance) {
			shortestDistance = testDistance;
		}
	}
	return shortestDistance;
}

// Calculates the shortest distance between the closest opponent and the expected trajectory of the ball
double PassPoint::calcDistOppToBallTraj(roboteam_utils::Vector2 testPosition, roboteam_msgs::World world) {
	roboteam_utils::Vector2 ballPos(world.ball.pos);
	roboteam_utils::Vector2 ballTraj = testPosition - ballPos;

	roboteam_utils::Vector2 oppPos = Vector2(world.them.at(0).pos);
	double shortestDistance = fabs((ballTraj.closestPointOnVector(ballPos, oppPos) - oppPos).length());
	for (size_t i = 1; i < world.them.size(); i++) {
		roboteam_utils::Vector2 oppPos = Vector2(world.them.at(i).pos);
		double testDistance = fabs((ballTraj.closestPointOnVector(ballPos, oppPos) - oppPos).length());
		if (testDistance < shortestDistance) {
			shortestDistance = testDistance;
		}
	}
	return shortestDistance;
}

// If multiple robots are blocking the view of the goal, and two of these robots are (partly) covering
// the same part of the goal, this functions combines these two robot cones into one cone
std::vector<Cone> PassPoint::combineOverlappingRobots(std::vector<Cone> robotCones) {
	for (size_t i = 0; i < robotCones.size(); i++) {
		Cone cone1 = robotCones.at(i);
		for (size_t j = 0; j < robotCones.size(); j++) {
			if (i!=j) {
				Cone cone2 = robotCones.at(j);
				if (cone1.DoConesOverlap(cone2)) {
					Cone newCone = cone1.MergeCones(cone2);

					std::vector<Cone> newRobotCones;
					newRobotCones.push_back(newCone);
					for (size_t k = 0; k < robotCones.size(); k++) {
						if (k!=i && k!=j) {
							newRobotCones.push_back(robotCones.at(k));
						}
					}
					return newRobotCones;
				}
			}
		}
	}
	return robotCones;
}

// Calculale the angular view of the goal, seen from the testPosition. Robots of the opposing team that are blocking
// the view, are taken into account
double PassPoint::calcViewOfGoal(roboteam_utils::Vector2 testPosition, roboteam_msgs::World world) {
	roboteam_utils::Vector2 theirGoal = LastWorld::get_their_goal_center();
	roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
	
	roboteam_utils::Vector2 vecToGoalSide1 = theirGoal + Vector2(0, -field.goal_width/2) - testPosition;
	roboteam_utils::Vector2 vecToGoalSide2 = theirGoal + Vector2(0, field.goal_width/2) - testPosition;

	Cone goalCone(testPosition, vecToGoalSide1, vecToGoalSide2);
	
	std::vector<Cone> robotCones;
	for (size_t i = 0; i < world.them.size(); i++) {
		if(goalCone.IsWithinCone(Vector2(world.them.at(i).pos), 0.09)) {
			Cone robotCone(testPosition, Vector2(world.them.at(i).pos), 0.09); 
			robotCones.push_back(robotCone);
		}
	}

	std::vector<Cone> combinedRobotCones = combineOverlappingRobots(robotCones);
	combinedRobotCones = combineOverlappingRobots(combinedRobotCones);

	double viewOfGoal = cleanAngle(vecToGoalSide1.angle() - vecToGoalSide2.angle());
	for (size_t i = 0; i < combinedRobotCones.size(); i++) {

		Cone robotCone = combinedRobotCones.at(i);

		double robotSide1Angle = cleanAngle(robotCone.side1.angle());
		double robotSide2Angle = cleanAngle(robotCone.side2.angle());
		double goalConeSide1Angle = cleanAngle(goalCone.side1.angle());
		double goalConeSide2Angle = cleanAngle(goalCone.side2.angle());

		if (isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide1Angle) && isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide2Angle)) { // robotCone completely within goalCone
			double blockedAngle = fabs(cleanAngle(robotSide1Angle - robotSide2Angle));
			viewOfGoal -= blockedAngle;
		} else if (isBetweenAngles(robotSide2Angle, robotSide1Angle, goalConeSide1Angle) && isBetweenAngles(robotSide2Angle, robotSide1Angle, goalConeSide2Angle)) { // goalCone completely within robotCone
			viewOfGoal = 0;
			break;
		} else if (isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide2Angle)) { // only on the left side 
			double blockedAngle = cleanAngle(goalCone.side1.angle() - robotCone.side2.angle());
			viewOfGoal -= blockedAngle;
		} else if (isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide1Angle)) { // only on the right side
			double blockedAngle = cleanAngle(robotCone.side1.angle() - goalCone.side2.angle());
			viewOfGoal -= blockedAngle;
		} else {
			ROS_INFO_STREAM("hmm, this is probably not right..." << testPosition.x << " " << testPosition.y);
		}
	}

	return viewOfGoal;
}

// Computes the score of a testPosisiton (higher score = better position to pass the ball to), based on a set of weights
double PassPoint::computePassPointScore(roboteam_utils::Vector2 testPosition) {
	roboteam_msgs::World world = LastWorld::get();
	while(world.us.size() == 0) {
		ros::spinOnce();
		world = LastWorld::get();
	}
	roboteam_utils::Vector2 ballPos(world.ball.pos);
	double distToGoal = (testPosition - LastWorld::get_their_goal_center()).length();
	double distToOpp = sqrt(calcDistToClosestOpp(testPosition, world));
	double distToBall = (testPosition - ballPos).length();
	double distOppToBallTraj = calcDistOppToBallTraj(testPosition, world);
	double viewOfGoal = sqrt(calcViewOfGoal(testPosition, world) / 0.336); // equals 1 when the angle is 0.336 radians, which is the view one meter in front of the goal

	double score = -distToGoal*distToGoalWeight + distToOpp*distToOppWeight - distToBall*distToBallWeight + viewOfGoal*viewOfGoalWeight + distOppToBallTraj*distOppToBallTrajWeight;
	return score;
}

// Checks many positions in the field and determines which has the highest score (higher score = better position to pass the ball to),
// also draws a 'heat map' in rqt_view
roboteam_utils::Vector2 PassPoint::computeBestPassPoint() {
	roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
	int x_steps = 27;
	int y_steps = 18;

	std::vector<roboteam_utils::Vector2> passPoints;
	std::vector<double> scores;
	std::vector<std::string> names;

	for (int x_step = 1; x_step < x_steps; x_step++) {
		double x = -field.field_length/2 + x_step*field.field_length/x_steps;
		for (int y_step = 1; y_step < y_steps; y_step++) {
			double y = -field.field_width/2 + y_step*field.field_width/y_steps;

			// calculate the score of this point:
			roboteam_utils::Vector2 point(x, y);

			if (!isWithinDefenseArea("their defense area", point)) {
				passPoints.push_back(point);
				double score = computePassPointScore(point);
				scores.push_back(score);

				// generate a name:
				std::string x_string = std::to_string(x_step);
				std::string y_string = std::to_string(y_step);
				std::string name = "point";
				name.append("x");
				name.append(x_string);
				name.append("y");
				name.append(y_string);
				names.push_back(name);
			}
		}
	}

	double maxScore = *max_element(scores.begin(), scores.end());
	double minScore = *min_element(scores.begin(), scores.end());

	for (size_t i = 0; i < passPoints.size(); i++) {
		double relScore = (scores.at(i) - minScore) / (maxScore - minScore) * 255;
		drawer.SetColor(255 - relScore, 0, relScore);
		drawer.DrawPoint(names.at(i), passPoints.at(i));
	}

	roboteam_utils::Vector2 bestPosition = passPoints.at(distance(scores.begin(), max_element(scores.begin(), scores.end())));
	return bestPosition;
}

} // rtt