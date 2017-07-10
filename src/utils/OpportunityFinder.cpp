#include "roboteam_tactics/utils/OpportunityFinder.h"
#include "roboteam_tactics/skills/ShootAtGoalV2.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/conditions/IsBallInDefenseArea.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "ros/package.h"

#include <iostream>
#include <fstream>


#define RTT_CURRENT_DEBUG_TAG OpportunityFinder
#define PASS_POINT_WEIGHTS_DIRECTORY ros::package::getPath("roboteam_tactics").append("/src/utils/OpportunityFinderWeights/")
#define DRAW_PASS_POINT_GRID false

namespace rtt {

OpportunityFinder::OpportunityFinder() {}

void OpportunityFinder::Initialize(std::string fileName, int ROBOT_ID, std::string target, int targetID) {

	std::string filePath = PASS_POINT_WEIGHTS_DIRECTORY.append(fileName);

	std::vector<float> weightsVector;
	std::string line;
	std::fstream myfile(filePath, std::ios_base::in);
	if (myfile.is_open()) {
		while (getline(myfile, line)) {
			size_t i = line.find(" ");
			std::string numberString = line.substr(i);
			weightsVector.push_back(std::stod(numberString));
		}

		myfile.close();

		distToGoalWeight = weightsVector.at(0);
		distToOppWeight = weightsVector.at(1);
		distToTeammateWeight = weightsVector.at(2);
		distToBallWeight = weightsVector.at(3);
		viewOfGoalWeight = weightsVector.at(4);
		distOppToBallTrajWeight = weightsVector.at(5);
		distOppToBallToTargetTrajWeight = weightsVector.at(6);
		distToRobotWeight = weightsVector.at(7);
		angleDiffRobotTargetWeight = weightsVector.at(8);
		distToRobotThreshold = weightsVector.at(9);
		distOppToBallTrajThreshold = weightsVector.at(10);
		distOppToBallToTargetTrajThreshold = weightsVector.at(11);
		viewOfGoalThreshold = weightsVector.at(12);
	} else {
		RTT_DEBUG("Unable to open file \n");
	}

	this->ROBOT_ID = ROBOT_ID;
	this->target = target;
	this->targetID = targetID;

	isCloseToPosSet = false;
}

// Calculates the distance between the closest opponent and the testPosition
double OpportunityFinder::calcDistToClosestOpp(Vector2 testPosition, roboteam_msgs::World world) {
	double shortestDistance = (Vector2(world.them.at(0).pos) - testPosition).length();
	for (size_t i = 1; i < world.them.size(); i++) {
		double testDistance = (Vector2(world.them.at(i).pos) - testPosition).length();
		if (testDistance < shortestDistance) {
			shortestDistance = testDistance;
		}
	}
	return shortestDistance;
}


// Calculates the distance between the closest opponent and the testPosition
double OpportunityFinder::calcDistToClosestTeammate(Vector2 testPosition, roboteam_msgs::World world) {
	double shortestDistance = (Vector2(world.us.at(0).pos) - testPosition).length();
	for (size_t i = 1; i < world.us.size(); i++) {
		double testDistance = (Vector2(world.us.at(i).pos) - testPosition).length();
		if (testDistance < shortestDistance) {
			shortestDistance = testDistance;
		}
	}
	return shortestDistance;
}

// Calculates the shortest distance between the closest opponent and the expected trajectory of the ball
double OpportunityFinder::calcDistOppToBallTraj(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 ballPos(world.ball.pos);
	Vector2 ballTraj = testPosition - ballPos;

	Vector2 oppPos = Vector2(world.them.at(0).pos);
	double shortestDistance = fabs((ballTraj.closestPointOnVector(ballPos, oppPos) - oppPos).length());
	for (size_t i = 1; i < world.them.size(); i++) {
		Vector2 oppPos = Vector2(world.them.at(i).pos);
		double testDistance = fabs((ballTraj.closestPointOnVector(ballPos, oppPos) - oppPos).length());
		if (testDistance < shortestDistance) {
			shortestDistance = testDistance;
		}
	}
	return shortestDistance;
}

// Calculates the shortest distance between the closest opponent and the expected trajectory of the ball
double OpportunityFinder::calcDistOppToBallToTargetTraj(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 ballToTargetTraj = targetPos - testPosition;

	Vector2 oppPos = Vector2(world.them.at(0).pos);
	double shortestDistance = fabs((ballToTargetTraj.closestPointOnVector(testPosition, oppPos) - oppPos).length());
	for (size_t i = 1; i < world.them.size(); i++) {
		Vector2 oppPos = Vector2(world.them.at(i).pos);
		double testDistance = fabs((ballToTargetTraj.closestPointOnVector(testPosition, oppPos) - oppPos).length());
		if (testDistance < shortestDistance) {
			shortestDistance = testDistance;
		}
	}
	return shortestDistance;
}


// If multiple robots are blocking the view of the goal, and two of these robots are (partly) covering
// the same part of the goal, this functions combines these two robot cones into one cone
std::vector<Cone> OpportunityFinder::combineOverlappingRobots(std::vector<Cone> robotCones) {
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
double OpportunityFinder::calcViewOfGoal(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 theirGoal = LastWorld::get_their_goal_center();
	roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
	
	Vector2 vecToGoalSide1 = theirGoal + Vector2(0, -field.goal_width/2) - testPosition;
	Vector2 vecToGoalSide2 = theirGoal + Vector2(0, field.goal_width/2) - testPosition;

	Cone goalCone(testPosition, vecToGoalSide2, vecToGoalSide1);

	std::vector<Cone> robotCones;
	for (size_t i = 0; i < world.them.size(); i++) {
		if(goalCone.IsWithinCone(Vector2(world.them.at(i).pos), 0.09)) {
			Cone robotCone(testPosition, Vector2(world.them.at(i).pos), 0.09); 
			robotCones.push_back(robotCone);
		}
	}

	std::vector<Cone> combinedRobotCones = combineOverlappingRobots(robotCones);
	combinedRobotCones = combineOverlappingRobots(combinedRobotCones);

	double viewOfGoal = cleanAngle(goalCone.side1.angle() - goalCone.side2.angle());

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

// Calculates the distance between the testPosition and the current robot
double OpportunityFinder::calcDistToRobot(Vector2 testPosition, roboteam_msgs::World world) {
	
	if (isCloseToPosSet) {
		return (testPosition - closeToPos).length();
	} else {
		boost::optional<roboteam_msgs::WorldRobot> bot = getWorldBot(ROBOT_ID, true, world);
		if (bot) {
			roboteam_msgs::WorldRobot robot = *bot;
			Vector2 robotPos(robot.pos);
			return (testPosition - robotPos).length();
		} else {
			ROS_WARN("OpportunityFinder::distToRobot robot not found :(");
			return 0.0;
		}
	}	
}


// Calculates the angle difference between the vector from the testPosition to the goal, and the vector from the testPosition to the ball
// If this angle is low, it means that a robot standing on the testPosition can more easily shoot the ball at the goal directly after receiving the ball
double OpportunityFinder::calcAngleDiffRobotTarget(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 ballPos(world.ball.pos);
	double angleDiffRobotTarget = fabs(cleanAngle((targetPos-testPosition).angle() - (ballPos-testPosition).angle()));
	return angleDiffRobotTarget;
}

void OpportunityFinder::setCloseToPos(Vector2 closeToPos) {
	this->closeToPos = closeToPos;
	isCloseToPosSet = true;
}

// Computes the score of a testPosisiton (higher score = better position to pass the ball to), based on a set of weights
// boost::optional<double> OpportunityFinder::computePassPointScore(Vector2 testPosition) {
double OpportunityFinder::computeScore(Vector2 testPosition) {
	roboteam_msgs::World world = LastWorld::get();

	// if (world.us.size() == 0) {
	// 	return boost::none;
	// }

	// double distToRobot = calcDistToRobot(testPosition, world);
	// if (distToRobot > distToRobotThreshold) {
	// 	return boost::none;
	// }

	
	double score = 0.0;

	double distOppToBallTraj = calcDistOppToBallTraj(testPosition, world);
	if (distOppToBallTraj < distOppToBallTrajThreshold) {
		score -= distOppToBallTrajWeight;
	}


	double distOppToBallToTargetTraj = calcDistOppToBallToTargetTraj(testPosition, world);
	if (distOppToBallToTargetTraj < distOppToBallToTargetTrajThreshold) {
		score -= distOppToBallToTargetTrajWeight;
	}

	// double viewOfGoal = sqrt(calcViewOfGoal(testPosition, world));
	// if (viewOfGoal < viewOfGoalThreshold) {
	// 	return boost::none;
	// }


	Vector2 ballPos(world.ball.pos);
	time_point currenTime = now();
	double distToGoal = (testPosition - LastWorld::get_their_goal_center()).length();
	double distToOpp = sqrt(calcDistToClosestOpp(testPosition, world));
	double distToTeammate = sqrt(calcDistToClosestTeammate(testPosition, world));
	double distToBall = (testPosition - ballPos).length();
	double viewOfGoal = calcViewOfGoal(testPosition, world) / 0.336 * distToGoal; // equals 1 when the angle is 0.336 radians, which is the view one meter in front of the goal
	// double distOppToBallTraj = calcDistOppToBallTraj(testPosition, world);
	double distToRobot = calcDistToRobot(testPosition, world);
	double angleDiffRobotTarget = calcAngleDiffRobotTarget(testPosition, world);



	angleDiffRobotTarget -= (60.0 / 180.0 * M_PI);
	angleDiffRobotTarget = fabs(angleDiffRobotTarget);

	// if (angleDiffRobotTarget <= (30.0 / 180.0 * M_PI)) {
		// angleDiffRobotTarget = M_PI;
	// }
	
	score += - distToGoal*distToGoalWeight 
				   + distToOpp*distToOppWeight 
				   + distToTeammate*distToTeammateWeight
				   + distToBall*distToBallWeight 
				   + viewOfGoal*viewOfGoalWeight
				   // + distOppToBallTraj*distOppToBallTrajWeight
				   - distToRobot*distToRobotWeight
				   - angleDiffRobotTarget*angleDiffRobotTargetWeight;

	return score;
}

// Checks many positions in the field and determines which has the highest score (higher score = better position to pass the ball to),
// also draws a 'heat map' in rqt_view
Vector2 OpportunityFinder::computeBestOpportunity() {
	
	roboteam_msgs::World world = LastWorld::get();
	targetPos = getTargetPos(target, targetID, true);

	int x_steps = 60;
	int y_steps = 40;

	// Remove all the old drawn points
	// for (int x_step = 1; x_step < x_steps; x_step++) {
	// 	for (int y_step = 1; y_step < y_steps; y_step++) {
	// 		// generate a name:
	// 		std::string x_string = std::to_string(x_step);
	// 		std::string y_string = std::to_string(y_step);
	// 		std::string name = "point";
	// 		name.append("x");
	// 		name.append(x_string);
	// 		name.append("y");
	// 		name.append(y_string);
	// 		drawer.removePoint(name);
	// 		// ros::spinOnce();
	// 	}
	// }
	// drawer.removePoint("bestPosition");

	roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

	std::vector<Vector2> opportunities;
	std::vector<double> scores;
	std::vector<std::string> names;

	std::string our_side;
    ros::param::get("our_side", our_side);

	for (int x_step = 1; x_step < x_steps; x_step++) {
		double x = -field.field_length/2 + x_step*field.field_length/x_steps;
		for (int y_step = 1; y_step < y_steps; y_step++) {
			double y = -field.field_width/2 + y_step*field.field_width/y_steps;

			// calculate the score of this point:
			Vector2 point(x, y);
			double dist = calcDistToRobot(point, world);
			Vector2 ballPos(world.ball.pos);
			double distToBall = (point - ballPos).length();

			if (dist < distToRobotThreshold && !isWithinDefenseArea(false, point) && distToBall >= 1.0) {
			// if (dist < distToRobotThreshold && !isWithinDefenseArea("their defense area", point)) {
				
				// boost::optional<double> score = computePassPointScore(point);
				double score = computeScore(point);
				// if (score) {
					opportunities.push_back(point);
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
				// }
			}
		}
	}

	if (opportunities.size() == 0) {
		ROS_WARN("No position found that meets the requirements :(");
		return Vector2(0.0, 0.0);
	}

	


	if (DRAW_PASS_POINT_GRID) {

		double maxScore = *max_element(scores.begin(), scores.end());
		double minScore = *min_element(scores.begin(), scores.end());
		for (size_t i = 0; i < opportunities.size(); i++) {
			double relScore = (scores.at(i) - minScore) / (maxScore - minScore) * 255;
			drawer.setColor(255 - relScore, 0, relScore);
			drawer.drawPoint(names.at(i), opportunities.at(i));
		}
	}

	Vector2 bestPosition = opportunities.at(distance(scores.begin(), max_element(scores.begin(), scores.end())));
	std::string winningPointName = names.at(distance(scores.begin(), max_element(scores.begin(), scores.end())));
	// drawer.removePoint(winningPointName);

	// std::string name = "bestPosition";
	// name.append(std::to_string(ROBOT_ID));
	// drawer.setColor(255, 255, 255);
	// drawer.drawPoint(name, bestPosition);
	return bestPosition;
}

} // rtt