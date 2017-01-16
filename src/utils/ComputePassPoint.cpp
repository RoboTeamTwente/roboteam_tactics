#include "roboteam_tactics/utils/ComputePassPoint.h"

#include "roboteam_tactics/utils/Math.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"

namespace rtt {

PassPoint::PassPoint() {}

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

std::vector<Cone> PassPoint::combineOverlappingRobots(std::vector<Cone> robotCones) {
	for (size_t i = 0; i < robotCones.size(); i++) {
		Cone cone1 = robotCones.at(i);
		for (size_t j = 0; j < robotCones.size(); j++) {
			if (i!=j) {
				Cone cone2 = robotCones.at(j);
				if (cone1.DoConesOverlap(cone2)) {
					ROS_INFO_STREAM("cones " << i << " and " << j << " overlap!");
					Cone newCone = cone1.MergeCones(cone2);
					robotCones.erase(robotCones.begin()+i);
					robotCones.erase(robotCones.begin()+j);
					robotCones.push_back(newCone);
					return robotCones;
				}
			}
		}
	}
	return robotCones;
}

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

	// ROS_INFO_STREAM("amount of robots in the way: " << robotCones.size());

	std::vector<Cone> combinedRobotCones = combineOverlappingRobots(robotCones);
	// combinedRobotCones = combineOverlappingRobots(combinedRobotCones);
	// while (combinedRobotCones.size() != robotCones.size()) {
	// 	ROS_INFO_STREAM("old size: " << combinedRobotCones.size());
	// 	robotCones = combinedRobotCones;
	// 	combinedRobotCones = combineOverlappingRobots(robotCones);
	// 	ROS_INFO_STREAM("new size: " << combinedRobotCones.size());
	// }

	if (combinedRobotCones.size() == 1) {
		drawer.SetColor(255, 255, 0);
		drawer.DrawLine("line1", testPosition, combinedRobotCones.at(0).side1);
		drawer.DrawLine("line2", testPosition, combinedRobotCones.at(0).side2);
		drawer.RemoveLine("line11");
		drawer.RemoveLine("line12");
		drawer.RemoveLine("line21");
		drawer.RemoveLine("line22");
	}
	if (combinedRobotCones.size() == 2) {
		drawer.SetColor(255, 255, 0);
		drawer.DrawLine("line1", testPosition, combinedRobotCones.at(0).side1);
		drawer.DrawLine("line2", testPosition, combinedRobotCones.at(0).side2);
		drawer.SetColor(255, 0, 0);
		drawer.DrawLine("line11", testPosition, combinedRobotCones.at(1).side1);
		drawer.DrawLine("line12", testPosition, combinedRobotCones.at(1).side2);
		drawer.RemoveLine("line21");
		drawer.RemoveLine("line22");
	}
	if (combinedRobotCones.size() == 3) {
		drawer.SetColor(255, 255, 0);
		drawer.DrawLine("line1", testPosition, combinedRobotCones.at(0).side1);
		drawer.DrawLine("line2", testPosition, combinedRobotCones.at(0).side2);
		drawer.SetColor(255, 0, 0);
		drawer.DrawLine("line11", testPosition, combinedRobotCones.at(1).side1);
		drawer.DrawLine("line12", testPosition, combinedRobotCones.at(1).side2);
		drawer.SetColor(255, 0, 255);
		drawer.DrawLine("line21", testPosition, combinedRobotCones.at(2).side1);
		drawer.DrawLine("line22", testPosition, combinedRobotCones.at(2).side2);
	}

	double viewOfGoal = cleanAngle(vecToGoalSide1.angle() - vecToGoalSide2.angle());
	for (size_t i = 0; i < robotCones.size(); i++) {

		Cone robotCone = robotCones.at(i);

		double robotSide1Angle = cleanAngle(robotCone.side1.angle());
		double robotSide2Angle = cleanAngle(robotCone.side2.angle());
		double goalConeSide1Angle = cleanAngle(goalCone.side1.angle());
		double goalConeSide2Angle = cleanAngle(goalCone.side2.angle());

		if (isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide1Angle) && isBetweenAngles(goalConeSide2Angle, goalConeSide1Angle, robotSide2Angle)) { // robotCone completely within goalCone
			double blockedAngle = fabs(cleanAngle(robotSide1Angle - robotSide2Angle));
			// ROS_INFO_STREAM("blockedAngle: " << blockedAngle);
			viewOfGoal -= blockedAngle;
		} else if (isBetweenAngles(robotSide2Angle, robotSide1Angle, goalConeSide1Angle) && isBetweenAngles(robotSide2Angle, robotSide1Angle, goalConeSide2Angle)) { // goalCone completely within robotCone
			viewOfGoal = 0;
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

double PassPoint::computePassPointScore(roboteam_utils::Vector2 testPosition) {
	roboteam_msgs::World world = LastWorld::get();
	while(world.us.size() == 0) {
		ros::spinOnce();
		world = LastWorld::get();
	}
	roboteam_utils::Vector2 ballPos(world.ball.pos);
	double distToGoal = (testPosition - LastWorld::get_their_goal_center()).length();
	double distToOpp = calcDistToClosestOpp(testPosition, world);
	double distToBall = (testPosition - ballPos).length();
	double distOppToBallTraj = calcDistOppToBallTraj(testPosition, world);
	double viewOfGoal = sqrt(sqrt(calcViewOfGoal(testPosition, world) / 0.336)); // equals 1 when the angle is 0.336 radians, which is the view one meter in front of the goal

	double score = -distToGoal*distToGoalWeight + distToOpp*distToOppWeight - distToBall*distToBallWeight + viewOfGoal*viewOfGoalWeight + distOppToBallTraj*distOppToBallTrajWeight;
	return score;
}

double PassPoint::computeBestPassPoint() {
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
	double avgScore = (maxScore + minScore) / 2;

	for (size_t i = 0; i < passPoints.size(); i++) {
		double relScore = (scores.at(i) - minScore) / (maxScore - minScore) * 255;
		drawer.SetColor(255 - relScore, 0, relScore);
		drawer.DrawPoint(names.at(i), passPoints.at(i));
	}
	return maxScore;
}

} // rtt