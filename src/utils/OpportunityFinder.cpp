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
#define DRAW_PASS_POINT_GRID false

namespace rtt {

OpportunityFinder::OpportunityFinder() {}

void OpportunityFinder::Initialize(std::string fileName, int ROBOT_ID, std::string target, int targetID) {

	ROS_INFO_STREAM_NAMED("utils.OpportunityFinder", "Initialize");

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
		// calculate totalWeight for normalization in computeScore
		totalWeight = viewOfGoalWeight + distToOppWeight + distToTeammateWeight
					+ distToBallWeight + distToSelfWeight + ballReflectionAngleWeight
					+ distOppToBallTrajWeight + distOppToTargetTrajWeight + distToGoalWeight + 0.0001;
	} else {
		ROS_WARN_STREAM_NAMED("utils.OpportunityFinder", "Unable to open file " << fileName);
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
	
	double shortestDistance = 40;//(Vector2(world.us.at(0).pos) - testPosition).length();
	for (size_t i = 0; i < world.us.size(); i++) {

		if (world.us.at(i).id!=ROBOT_ID){ // I should not check my own position

			double testDistance = (Vector2(world.us.at(i).pos) - testPosition).length();
			if (testDistance < shortestDistance) {
				shortestDistance = testDistance;
			}
		}
	}
	return shortestDistance;
}

// Calculates the distance of the closest opponent to the expected trajectory of the ball
double OpportunityFinder::calcDistOppToBallTraj(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 ballPos(world.ball.pos);
	Vector2 ballTraj = testPosition - ballPos;

	if (world.them.size() == 0) {
		return 40.0;
	}
	// IMPROVEMENT: Maybe the ball trajectory should be considered a cone, when assessing how close opponents may be.
	// Vector2 oppPos = Vector2(world.them.at(0).pos);
	double shortestDistance = 40;//fabs((ballTraj.closestPointOnVector(ballPos, oppPos) - oppPos).length());
	for (size_t i = 0; i < world.them.size(); i++) {
		Vector2 oppPos(world.them.at(i).pos);
		// WIP: opponents outside a 90 degree field of view (as seen from the ball) are not considered
		if ( fabs(cleanAngle( (oppPos - ballPos).angle() - ballTraj.angle() )) < 0.25*M_PI) {
			double testDistance = (ballTraj.closestPointOnVector(ballPos, oppPos) - oppPos).length();
			if (testDistance < shortestDistance) {
				shortestDistance = testDistance;
			}
		}
	}
	return shortestDistance;
}

// Calculates the distance of the closest opponent to the expected trajectory from testPosition to target
double OpportunityFinder::calcDistOppToTargetTraj(Vector2 testPosition, roboteam_msgs::World world) {
	Vector2 posToTargetTraj = targetPos - testPosition;

	if (world.them.size() == 0) {
		return 40.0;
	}

	// IMPROVEMENT: Maybe the trajectory should be considered a cone, when assessing how close opponents may be.
	double shortestDistance = 40;
	for (size_t i = 0; i < world.them.size(); i++) {
		Vector2 oppPos(world.them.at(i).pos);
		// WIP: opponents outside a 90 degree field of view (as seen from the testPosition) are not considered
		if ( fabs(cleanAngle( (oppPos - testPosition).angle() - posToTargetTraj.angle() )) < 0.25*M_PI) {
			double testDistance = (posToTargetTraj.closestPointOnVector(testPosition, oppPos) - oppPos).length();
			if (testDistance < shortestDistance) {
				shortestDistance = testDistance;
			}
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
			return (testPosition - Vector2((*bot).pos)).length();
			
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


double OpportunityFinder::computeScore(Vector2 testPosition) {
	roboteam_msgs::World world = LastWorld::get();
	return computeScore(testPosition, world);
}
// Computes the score of a testPosisiton (higher score = better position to pass the ball to), based on a set of weights
// boost::optional<double> OpportunityFinder::computePassPointScore(Vector2 testPosition) {
double OpportunityFinder::computeScore(Vector2 testPosition, roboteam_msgs::World world) {
	
	Vector2 ballPos(world.ball.pos);
	
	double score = 0.0;	// score will not go below 0

	if (distOppToBallTrajWeight>0.0) { // PRIORITY: ZERO SCORE IN THIS METRIC DIRECTLY LEADS TO OVERALL ZERO SCORE
		double distOppToBallTraj = calcDistOppToBallTraj(testPosition, world);
		if (distOppToBallTraj<distOppToBallTrajMin) {
			return 0.0;
		}
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

	if (distToGoalWeight>0.0) { // IMPROVEMENT: Distance from goal line, instead of goal center point?
		double distToGoal = (testPosition - targetPos).length();
		// Normalize such that 0 corresponds to best and 1 to worst possible score
		distToGoal = (distToGoal-distToGoalMin)/(distToGoalMax-distToGoalMin);
		// Add score to total score
		score += smoothStep(1-distToGoal)*distToGoalWeight;
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
		score += smoothStep(1-distToSelf)*distToSelfWeight;
	}
	
	if (ballReflectionAngleWeight>0.0) {
		// double ballReflectionAngle = calcBallReflectionAngle(testPosition, world);
		double ballReflectionAngle = fabs(cleanAngle((targetPos-testPosition).angle() - (ballPos-testPosition).angle()));
		// Normalize such that 0 corresponds to best and 1 to worst possible score
		ballReflectionAngle = (ballReflectionAngle-ballReflectionAngleMin)/(ballReflectionAngleMax-ballReflectionAngleMin);
		// Add score to total score
		score += smoothStep(1-ballReflectionAngle)*ballReflectionAngleWeight;
	}
		
	// normalize score between 0 and 100
	score = score / totalWeight * 100;

	return score;
}

// Checks many positions in the field and determines which has the highest score (higher score = better position to pass the ball to),
// also draws a 'heat map' in rqt_view
Vector2 OpportunityFinder::computeBestOpportunity(Vector2 centerPoint, double boxLength, double boxWidth) {
	
	roboteam_msgs::World world = LastWorld::get();
	// time_point start = now();
	
	int x_steps = 20;
	int y_steps = 20;

	// Get claimed positions, place them instead of those robot positions in our world object
	for (size_t i = 0; i < world.us.size(); i++) {
		// check whether current bot claimed a position
		double botClaimedX;
		ros::param::getCached("robot" + std::to_string(world.us.at(i).id) + "/claimedPosX", botClaimedX);
		if( !(botClaimedX == 0.0 && std::signbit(botClaimedX)) ) { // if not -0.0, bot actually claimed a position
			double botClaimedY;
			ros::param::getCached("robot" + std::to_string(world.us.at(i).id) + "/claimedPosY", botClaimedY);
			world.us.at(i).pos.x = float(botClaimedX);
			world.us.at(i).pos.y = float(botClaimedY);
		}
	}

	std::vector<Vector2> opportunities;
	std::vector<double> scores;
	std::vector<std::string> names;

	for (int x_step = 1; x_step < x_steps; x_step++) {
		double x = -(boxLength)/2 + x_step*(boxLength)/x_steps + centerPoint.x;
		for (int y_step = 1; y_step < y_steps; y_step++) {
			double y = -(boxWidth)/2 + y_step*(boxWidth)/y_steps + centerPoint.y;
			Vector2 point(x, y);
			// generate a name:
				std::string name = "pointx";
				name.append(std::to_string(x_step));
				name.append("y");
				name.append(std::to_string(y_step));
			// calculate the score of this point:
			if (!isWithinDefenseArea(false, point, 0.2) && IsWithinField(point)) {
				double score = computeScore(point, world);
				opportunities.push_back(point);
				scores.push_back(score);
				names.push_back(name);
			} else if (DRAW_PASS_POINT_GRID){ // points not within bounds are reset in drawing
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

	// int timePassed = time_difference_milliseconds(start, now()).count();
	//ROS_INFO_STREAM("robot: " << ROBOT_ID << " OppFinder took: " << timePassed << " ms");

	// Info about best position
	// ROS_INFO_STREAM("viewOfGoal: " << calcViewOfGoal(bestPosition, world));
	// ROS_INFO_STREAM("distClosestTeammate" << calcDistToClosestTeammate(bestPosition, world));
	std::string name = "bestPosition";
	name.append(std::to_string(ROBOT_ID));
	drawer.setColor(255, 255, 255);
	drawer.drawPoint(name, bestPosition);
	return bestPosition;
}


BestTeammate OpportunityFinder::chooseBestTeammate(bool realScore, bool realPos, bool notBackwards, bool distFromTargetLimit, double limit) {

	roboteam_msgs::World world = LastWorld::get();
	roboteam_msgs::World fakeWorld = world;
	

	if (!realScore || !realPos) {
		// Get claimed positions, place them instead of those robot positions in our 'fake' world object
		for (size_t i = 0; i < fakeWorld.us.size(); i++) {
			double botClaimedX;
			ros::param::getCached("robot" + std::to_string(fakeWorld.us.at(i).id) + "/claimedPosX", botClaimedX);
			if( !(botClaimedX == 0.0 && std::signbit(botClaimedX)) ) { // if not -0.0, bot actually claimed a position
				double botClaimedY;
				ros::param::getCached("robot" + std::to_string(fakeWorld.us.at(i).id) + "/claimedPosY", botClaimedY);
				fakeWorld.us.at(i).pos.x = float(botClaimedX);
				fakeWorld.us.at(i).pos.y = float(botClaimedY);
			}
		}
	}


	// initialize max score and corresponding ID and position
   	double maxScore = 0;
   	int bestID = -1;
   	Vector2 bestPos(0,0);
    
    Vector2 ballPos(fakeWorld.ball.pos);

    // For each of our robots
    for (size_t i = 0; i < (fakeWorld.us.size()); i++) {
        if (fakeWorld.us.at(i).id != ROBOT_ID) { // dont choose myself
        	Vector2 botPos;
        	if (realPos) {
        		botPos = Vector2(world.us.at(i).pos);
        	} else {
        		botPos = Vector2(fakeWorld.us.at(i).pos);
        	}
        	
        	if (notBackwards && botPos.x < ballPos.x) {
        	// if notBackwards true, robots too far from target will be skipped
        		continue;
        	}
        	if (distFromTargetLimit && (targetPos - botPos).length() < limit) {
        	// if distFromTargetLimit true, robots too far from target will be skipped
        		continue;
        	}
            double score;
            if (realScore) {
            	score = computeScore(botPos, world);
            } else {
            	score = computeScore(botPos,fakeWorld);
            } 
            if (score > maxScore) {
                maxScore = score;
                bestID = fakeWorld.us.at(i).id;
                bestPos = botPos;
            }
        }
    } // end of for-loop
    
    BestTeammate bestTeammate;
    bestTeammate.id = bestID;
    bestTeammate.pos = bestPos;

    if (maxScore > 0) {
        ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << ROBOT_ID << " used OpportunityFinder to choose " << bestID <<" as best teammate to pass to, with score: " << maxScore);
    } else {
        ROS_WARN_STREAM_NAMED("jelle_test1", "robot " << ROBOT_ID << " could not find an appropriate teammate to pass to using OpportunityFinder");
    }

    return bestTeammate;
}

} // rtt
