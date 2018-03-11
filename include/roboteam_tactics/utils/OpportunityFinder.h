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

class OpportunityFinder {

public:
	OpportunityFinder();
	void Initialize(std::string fileName, int ROBOT_ID, std::string target, int targetID);
	double calcDistToClosestOpp(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistToClosestTeammate(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistOppToBallTraj(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistOppToBallToTargetTraj(Vector2 testPosition, roboteam_msgs::World world);
	std::vector<Cone> combineOverlappingRobots(std::vector<Cone> robotCones);
	double calcViewOfGoal(Vector2 testPosition, roboteam_msgs::World world);
	std::pair<std::vector<double>, std::vector<double>> getOpenGoalAngles(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistToRobot(Vector2 testPosition, roboteam_msgs::World world);
	double calcAngleDiffRobotTarget(Vector2 testPosition, roboteam_msgs::World world);

	void setCloseToPos(Vector2 closeToPos);
	// boost::optional<double> computePassPointScore(Vector2 testPosition);
	double computeScore(Vector2 testPosition);
	Vector2 computeBestOpportunity(Vector2 centerPoint, double boxLength, double boxWidth);
private:

	// Weights for determining the score of a point on the field
	double distToGoalWeight;
	double distToOppWeight;
	double distToTeammateWeight;
	double distToBallWeight;
	double viewOfGoalWeight;
	double distOppToBallTrajWeight;
	double distOppToBallToTargetTrajWeight;

	double distToRobotWeight;
	double angleDiffRobotTargetWeight;

	// Thresholds for determining whether a point on the field should be considered or not
	double distToRobotThreshold;
	double distOppToBallTrajThreshold;
	double distOppToBallToTargetTrajThreshold;
	double viewOfGoalThreshold;

	// The ID of the robot for which are trying to find a good point to stand on the field
	uint ROBOT_ID;
	std::string target;
	int targetID;
	Vector2 targetPos;

	bool isCloseToPosSet;
	Vector2 closeToPos;

	Draw drawer;
	std::vector<std::string> names;
};

} // rtt
