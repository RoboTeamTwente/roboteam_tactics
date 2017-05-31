#pragma once

#include "ros/ros.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_utils/Cone.h"

#include <boost/optional.hpp>
#include <vector>
#include <string>

namespace rtt {

class PassPoint {

public:
	PassPoint();
	void Initialize(std::string fileName, int ROBOT_ID, std::string target, int targetID);
	double calcDistToClosestOpp(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistOppToBallTraj(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistOppToBallToTargetTraj(Vector2 testPosition, roboteam_msgs::World world);
	std::vector<Cone> combineOverlappingRobots(std::vector<Cone> robotCones);
	double calcViewOfGoal(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistToRobot(Vector2 testPosition, roboteam_msgs::World world);
	double calcAngleDiffRobotTarget(Vector2 testPosition, roboteam_msgs::World world);

	boost::optional<double> computePassPointScore(Vector2 testPosition);
	Vector2 computeBestPassPoint();
private:

	// Weights for determining the score of a point on the field
	double distToGoalWeight;
	double distToOppWeight;
	double distToBallWeight;
	double viewOfGoalWeight;
	double distOppToBallTrajWeight;
	double distToRobotWeight;
	double angleDiffRobotTargetWeight;

	// Thresholds for determining whether a point on the field should be considered or not
	double distToRobotThreshold;
	double distOppToBallTrajThreshold;
	double distOppToBallToTargetTrajThreshold;

	// The ID of the robot for which are trying to find a good point to stand on the field
	int ROBOT_ID;
	std::string target;
	int targetID;
	Vector2 targetPos;

	Draw drawer;
};

} // rtt