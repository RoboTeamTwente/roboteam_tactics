#include "roboteam_tactics/utils/OpportunityFinder.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/skills/ShootAtGoalV2.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/conditions/IsBallInDefenseArea.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "ros/package.h"

#include <iostream>
#include <fstream>


#define RTT_CURRENT_DEBUG_TAG OpportunityFinder
#define PASS_POINT_WEIGHTS_DIRECTORY ros::package::getPath("roboteam_tactics").append("/src/utils/OpportunityFinderWeights/")
#define DRAW_PASS_POINT_GRID true

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
		distToGoalMin = weightsVector.at(1);
		distToGoalMax = weightsVector.at(2);
		viewOfGoalWeight = weightsVector.at(3);
		viewOfGoalMin = weightsVector.at(4);
		viewOfGoalMax = weightsVector.at(5);
		distToOppWeight = weightsVector.at(6);
		distToOppMin = weightsVector.at(7);
		distToOppMax = weightsVector.at(8);
		distToTeammateWeight = weightsVector.at(9);
		distToTeammateMin = weightsVector.at(10);
		distToTeammateMax = weightsVector.at(11);
		distToBallWeight = weightsVector.at(12);
		distToBallMin = weightsVector.at(13);
		distToBallMax = weightsVector.at(14);
		distToSelfWeight = weightsVector.at(15);
		distToSelfMin = weightsVector.at(16);
		distToSelfMax = weightsVector.at(17);
		ballReflectionAngleWeight = weightsVector.at(18);
		ballReflectionAngleMin = weightsVector.at(19);
		ballReflectionAngleMax = weightsVector.at(20);
		distOppToBallTrajWeight = weightsVector.at(21);
		distOppToBallTrajMin = weightsVector.at(22);
		distOppToBallTrajMax = weightsVector.at(23);
		distOppToTargetTrajWeight = weightsVector.at(24);
		distOppToTargetTrajMin = weightsVector.at(25);
		distOppToTargetTrajMax = weightsVector.at(26);
	} else {
		RTT_DEBUG("Unable to open file \n");
	}

	this->ROBOT_ID = ROBOT_ID;
	this->target = target;
	this->targetID = targetID;
	targetPos = getTargetPos(target, targetID, true);

	isCloseToPosSet = false;
}

// Calculates the distance between the closest opponent and the testPosition
double OpportunityFinder::calcDistToClosestOpp(Vector2 testPosition, roboteam_msgs::World world) {

	if (world.them.size() == 0) {
		return 0.0;
	}

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
	
	double shortestDistance = 1000;
	for (size_t i = 0; i < world.us.size(); i++) {

		if (world.us.at(i).id!=ROBOT_ID){
			// check whether current bot claimed a position
			double botClaimedX;
			ros::param::getCached("robot" + std::to_string(world.us.at(i).id) + "/claimedPosX", botClaimedX);
			double testDistance = 0;
			if(botClaimedX == 0 && !std::signbit(botClaimedX)) { // check for a -0.0, meaning bot did not claim position
				testDistance = (Vector2(world.us.at(i).pos) - testPosition).length();
			} else { // if bot did claim a position, this is considered instead of its actual position
				double botClaimedY;
				ros::param::getCached("robot" + std::to_string(world.us.at(i).id) + "/claimedPosY", botClaimedY);
				testDistance = (Vector2(botClaimedX,botClaimedY) - testPosition).length();
			}
			// testDistance = (Vector2(world.us.at(i).pos) - testPosition).length();
			if (testDistance < shortestDistance) {
				shortestDistance = testDistance;
			}
		}
	}
	return shortestDistance;
}

// Calculates the shortest distance between the closest opponent and the expected trajectory of the ball
double OpportunityFinder::calcDistOppToBallTraj(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 ballPos(world.ball.pos);
	Vector2 ballTraj = testPosition - ballPos;

	if (world.them.size() == 0) {
		return 0.0;
	}

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
double OpportunityFinder::calcDistOppToTargetTraj(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 ballToTargetTraj = targetPos - testPosition;

	if (world.them.size() == 0) {
		return 0.0;
	}

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
	double viewOfGoal = 0.0;
	std::pair<std::vector<double>, std::vector<double>> openAngles = getOpenGoalAngles(testPosition, world);
	for (size_t i = 0; i < openAngles.second.size(); i++) {
   		viewOfGoal += openAngles.second.at(i) - openAngles.first.at(i);
	}
	// Vector2 theirGoal = LastWorld::get_their_goal_center();
	// roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
	
	// Vector2 vecToGoalSide1 = theirGoal + Vector2(0, -field.goal_width/2) - testPosition;
	// Vector2 vecToGoalSide2 = theirGoal + Vector2(0, field.goal_width/2) - testPosition;

	// Cone goalCone(testPosition, vecToGoalSide2, vecToGoalSide1);

	// std::vector<Cone> robotCones;
	// for (size_t i = 0; i < world.them.size(); i++) {
	// 	if(goalCone.IsWithinCone(Vector2(world.them.at(i).pos), 0.09)) {
	// 		Cone robotCone(testPosition, Vector2(world.them.at(i).pos), 0.09); 
	// 		robotCones.push_back(robotCone);
	// 	}
	// }

	// std::vector<Cone> combinedRobotCones = combineOverlappingRobots(robotCones);
	// combinedRobotCones = combineOverlappingRobots(combinedRobotCones);

	// double viewOfGoal = cleanAngle(goalCone.side1.angle() - goalCone.side2.angle());

	// for (size_t i = 0; i < combinedRobotCones.size(); i++) {

	// 	Cone robotCone = combinedRobotCones.at(i);
	// 	double robotSide1Angle = cleanAngle(robotCone.side1.angle());
	// 	double robotSide2Angle = cleanAngle(robotCone.side2.angle());
	// 	double goalConeSide1Angle = cleanAngle(goalCone.side1.angle());
	// 	double goalConeSide2Angle = cleanAngle(goalCone.side2.angle());

	// 	if (isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide1Angle) && isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide2Angle)) { // robotCone completely within goalCone
	// 		double blockedAngle = fabs(cleanAngle(robotSide1Angle - robotSide2Angle));
	// 		viewOfGoal -= blockedAngle;
	// 	} else if (isBetweenAngles(robotSide2Angle, robotSide1Angle, goalConeSide1Angle) && isBetweenAngles(robotSide2Angle, robotSide1Angle, goalConeSide2Angle)) { // goalCone completely within robotCone
	// 		viewOfGoal = 0;
	// 		break;
	// 	} else if (isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide2Angle)) { // only on the left side 
	// 		double blockedAngle = cleanAngle(goalCone.side1.angle() - robotCone.side2.angle());
	// 		viewOfGoal -= blockedAngle;
	// 	} else if (isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide1Angle)) { // only on the right side
	// 		double blockedAngle = cleanAngle(robotCone.side1.angle() - goalCone.side2.angle());
	// 		viewOfGoal -= blockedAngle;
	// 	} else {
	// 		ROS_INFO_STREAM("hmm, this is probably not right..." << testPosition.x << " " << testPosition.y);
 //            viewOfGoal = 0;
	// 	}
	// }
	
	return viewOfGoal;
}

std::pair<std::vector<double>, std::vector<double>> OpportunityFinder::getOpenGoalAngles(Vector2 testPosition, roboteam_msgs::World world) {
	// targetPos is one of the goals. IMPROVEMENT: Maybe add functionality that uses other views than only goal views (like view of dangerous positions)
	roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
	
	Vector2 goalSide1 = targetPos + Vector2(0, -field.goal_width/2+0.023); // 0.023 = ball radius
	Vector2 goalSide2 = targetPos + Vector2(0, field.goal_width/2-0.023);
	Vector2 vecToGoalSide1 = goalSide1 - testPosition;
	Vector2 vecToGoalSide2 = goalSide2 - testPosition;

	Cone goalCone(testPosition, vecToGoalSide1, vecToGoalSide2);
	std::vector<double> openAngles1;
	std::vector<double> openAngles2;

	// The goal angles are normalized (such that angle1 becomes 0 and angle 2 positive)..
	double goalAngle2 = fabs(cleanAngle(vecToGoalSide2.angle() - vecToGoalSide1.angle()));
	// ..and placed in the two 'open angle vectors', representing the initial open angle
	openAngles1.push_back(0);
	openAngles2.push_back(goalAngle2);

	// The open angle vectors are adjusted for each robot blocking sight of the goal.
	for (size_t i = 0; i < world.them.size(); i++) {
		Vector2 botPos(world.them.at(i).pos);
		if(goalCone.IsWithinCone(botPos, 0.113)) { // if robot blocks the goal, where 0.113 = robotRadius+ballRadius
			Vector2 vecToBot = botPos - testPosition;
			// botAngle: angle to center of blocking robot, normalized
			double botAngle = fabs(cleanAngle( vecToBot.angle() - vecToGoalSide1.angle() ));
			double blockedAngle = atan(0.113/vecToBot.length());
			double blocked1 = botAngle-blockedAngle; // startpoint of blocked angle
			double blocked2 = botAngle+blockedAngle; // endpoint of blocked angle
			size_t j = 0;
			while (j < openAngles1.size()) {
				double open1 = openAngles1.at(j); // startpoint of this open angle
				double open2 = openAngles2.at(j); // endpoint of this open angle

				if (blocked2 <= open1 || blocked1 >= open2) {
					// no blocking of current open angle -> next j
					j++;
				} else {
					if (blocked2 < open2) {
						if (blocked1 > open1) {
							// blocking is entirely within this open angle
							openAngles1.insert(openAngles1.begin()+j+1, blocked2);
							openAngles2.insert(openAngles2.begin()+j, blocked1);
							break; // exit the while loop
							ROS_INFO_STREAM("exitwhileloop");
						} else {
							// only the startpoint of this open angle is blocked
							openAngles1[j] = blocked2;
				 			j++;
						}
					} else {
						if (blocked1 > open1) {
							// only the endpoint of this open angle is blocked
							openAngles2[j] = blocked1;
				 			j++;
						} else {
							// this complete open angle is blocked -> delete and remain at same j
							openAngles1.erase(openAngles1.begin() + j);
							openAngles2.erase(openAngles2.begin() + j);
						}
					}
				}
			} // scan over and update the open angles
		} // if robot in goalCone
	} // for each opp. robot
	return {openAngles1,openAngles2}; // IMPROVEMENT: de-normalize angles again, for practical use.
}

// Calculates the distance between the testPosition and the current robot
double OpportunityFinder::calcDistToSelf(Vector2 testPosition, roboteam_msgs::World world) {
	
	if (isCloseToPosSet) {
		return (testPosition - closeToPos).length();
	} else {
		boost::optional<roboteam_msgs::WorldRobot> bot = getWorldBot(ROBOT_ID, true, world);
		if (bot) {
			// check whether I claimed a position
			double botClaimedX;
			ros::param::getCached("robot" + std::to_string(ROBOT_ID) + "/claimedPosX", botClaimedX);
			if(botClaimedX == 0 && !std::signbit(botClaimedX)) { // check for a -0.0, meaning bot did not claim position
				roboteam_msgs::WorldRobot robot = *bot;
				Vector2 robotPos(robot.pos);
				return (testPosition - robotPos).length();
			} else { // if bot did claim a position, this is considered instead of its actual position
				double botClaimedY;
				ros::param::getCached("robot" + std::to_string(ROBOT_ID) + "/claimedPosY", botClaimedY);
				return (Vector2(botClaimedX,botClaimedY) - testPosition).length();
			}
			
		} else {
			ROS_WARN("OpportunityFinder::distToRobot robot not found :(");
			return 0.0;
		}
	}	
}


// Calculates the angle difference between the vector from the testPosition to the goal, and the vector from the testPosition to the ball
// If this angle is low, it means that a robot standing on the testPosition can more easily shoot the ball at the goal directly after receiving the ball
double OpportunityFinder::calcBallReflectionAngle(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 ballPos(world.ball.pos);
	double ballReflectionAngle = fabs(cleanAngle((targetPos-testPosition).angle() - (ballPos-testPosition).angle()));
	return ballReflectionAngle;
}

void OpportunityFinder::setCloseToPos(Vector2 closeToPos) {
	this->closeToPos = closeToPos;
	isCloseToPosSet = true;
}

// Computes the score of a testPosisiton (higher score = better position to pass the ball to), based on a set of weights
// boost::optional<double> OpportunityFinder::computePassPointScore(Vector2 testPosition) {
double OpportunityFinder::computeScore(Vector2 testPosition) {
	roboteam_msgs::World world = LastWorld::get();
	Vector2 ballPos(world.ball.pos);

	// if (world.us.size() == 0) {
	// 	return boost::none;
	// }
	
	double score = 0.0;

	// POSSIBLE IMPROVEMENT: certain metrics have such priority that a zero score in these metrics will return a 0 (or negative? or invalid?) score immediately

	if (distToGoalWeight>0.0) { // IMPROVEMENT: Distance from goal line, instead of goal center point?
		double distToGoal = (testPosition - targetPos).length();
		// Normalize such that 0 corresponds to best and 1 to worst possible score
		distToGoal = (distToGoal-distToGoalMin)/(distToGoalMax-distToGoalMin);
		// Add score to total score
		score -= smoothStep(distToGoal)*distToGoalWeight;
	}

	if (viewOfGoalWeight>0.0){
		double viewOfGoal = calcViewOfGoal(testPosition, world);
		// Normalize such that 0 corresponds to worst and 1 to best possible score
		viewOfGoal = (viewOfGoal-viewOfGoalMin)/(viewOfGoalMax-viewOfGoalMin);
		// Add score to total score
		score += smoothStep(viewOfGoal)*viewOfGoalWeight;
	}

	if (distToOppWeight>0.0) {
		double distToOpp = calcDistToClosestOpp(testPosition, world);
		// Normalize such that 0 corresponds to worst and 1 to best possible score
		distToOpp = (distToOpp-distToOppMin)/(distToOppMax-distToOppMin);
		// Add score to total score
		score += smoothStep(distToOpp)*distToOppWeight;
	}

	if (distToTeammateWeight>0.0) {
		double distToTeammate = calcDistToClosestTeammate(testPosition, world);
		// ROS_INFO_STREAM("distToTeammate: " << distToTeammate);
		// Normalize such that 0 corresponds to worst and 1 to best possible score
		distToTeammate = (distToTeammate-distToTeammateMin)/(distToTeammateMax-distToTeammateMin);
		// Add score to total score
		score += smoothStep(distToTeammate)*distToTeammateWeight;
	}

	if (distToBallWeight>0.0) {
		double distToBall = (testPosition - ballPos).length();
		// Normalize such that 0 corresponds to worst and 1 to best possible score
		distToBall = (distToBall-distToBallMin)/(distToBallMax-distToBallMin);
		// Add score to total score
		score += smoothStep(distToBall)*distToBallWeight;
	}

	if (distToSelfWeight>0.0) {
		double distToSelf = calcDistToSelf(testPosition, world);
		// Normalize such that 0 corresponds to best and 1 to worst possible score
		distToSelf = (distToSelf-distToSelfMin)/(distToSelfMax-distToSelfMin);
		// Add score to total score
		score -= smoothStep(distToSelf)*distToSelfWeight;
	}
	
	if (ballReflectionAngleWeight>0.0) {
		double ballReflectionAngle = calcBallReflectionAngle(testPosition, world);
		// Normalize such that 0 corresponds to best and 1 to worst possible score
		ballReflectionAngle = (ballReflectionAngle-ballReflectionAngleMin)/(ballReflectionAngleMax-ballReflectionAngleMin);
		// Add score to total score
		score -= smoothStep(ballReflectionAngle)*ballReflectionAngleWeight;
	}

	if (distOppToBallTrajWeight>0.0) {
		double distOppToBallTraj = calcDistOppToBallTraj(testPosition, world);
		// Normalize such that 0 corresponds to worst and 1 to best possible score
		distOppToBallTraj = (distOppToBallTraj-distOppToBallTrajMin)/(distOppToBallTrajMax-distOppToBallTrajMin);
		// Add score to total score
		score += smoothStep(distOppToBallTraj)*distOppToBallTrajWeight;
	}

	if (distOppToTargetTrajWeight>0.0) {
		double distOppToTargetTraj = calcDistOppToTargetTraj(testPosition, world);
		// Normalize such that 0 corresponds to worst and 1 to best possible score
		distOppToTargetTraj = (distOppToTargetTraj-distOppToTargetTrajMin)/(distOppToTargetTrajMax-distOppToTargetTrajMin);
		// Add score to total score
		score += smoothStep(distOppToTargetTraj)*distOppToTargetTrajWeight;
	}

	return score;
}

// Checks many positions in the field and determines which has the highest score (higher score = better position to pass the ball to),
// also draws a 'heat map' in rqt_view
Vector2 OpportunityFinder::computeBestOpportunity(Vector2 centerPoint, double boxLength, double boxWidth) {

	// time_point start = now();
	
	roboteam_msgs::World world = LastWorld::get();

	int x_steps = 20;
	int y_steps = 20;

	std::vector<Vector2> opportunities;
	std::vector<double> scores;
	std::vector<std::string> names;

	std::string our_side;
    ros::param::get("our_side", our_side);


	for (int x_step = 1; x_step < x_steps; x_step++) {
		double x = -(boxLength)/2 + x_step*(boxLength)/x_steps + centerPoint.x;
		for (int y_step = 1; y_step < y_steps; y_step++) {
			double y = -(boxWidth)/2 + y_step*(boxWidth)/y_steps + centerPoint.y;
			// calculate the score of this point:
			Vector2 point(x, y);

			// generate a name:
				std::string x_string = std::to_string(x_step);
				std::string y_string = std::to_string(y_step);
				std::string name = "point";
				name.append("x");
				name.append(x_string);
				name.append("y");
				name.append(y_string);

			if (!isWithinDefenseArea(false, point, 0.1) && IsWithinField(point)) {
				double score = computeScore(point);
				opportunities.push_back(point);
				scores.push_back(score);
				names.push_back(name);
			} else {
				drawer.setColor(0,0,0);
				drawer.drawPoint(name, Vector2(0.0,0.0));
			}
			
		}
	}
	// int timePassed = time_difference_milliseconds(start, now()).count();
	// ROS_INFO_STREAM("robot: " << ROBOT_ID << " OppFinderBeforeDrawing took: " << timePassed << " ms");

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
	// std::string winningPointName = names.at(distance(scores.begin(), max_element(scores.begin(), scores.end())));

	// timePassed = time_difference_milliseconds(start, now()).count();
	// ROS_INFO_STREAM("robot: " << ROBOT_ID << " OppFinder took: " << timePassed << " ms");

	// Info about best position
	// ROS_INFO_STREAM("viewOfGoal: " << calcViewOfGoal(bestPosition, world));

	// std::string name = "bestPosition";
	// name.append(std::to_string(ROBOT_ID));
	// drawer.setColor(255, 255, 255);
	// drawer.drawPoint(name, bestPosition);
	return bestPosition;
}

} // rtt
