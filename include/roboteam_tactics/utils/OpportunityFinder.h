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

struct BestTeammate {
    Vector2 pos;
    int id;
};

class OpportunityFinder {

public:
	OpportunityFinder();
	void Initialize(std::string fileName, int ROBOT_ID, std::string target, int targetID);
	double calcDistToClosestOpp(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistToClosestTeammate(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistOppToBallTraj(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistOppToTargetTraj(Vector2 testPosition, roboteam_msgs::World world);
	std::vector<Cone> combineOverlappingRobots(std::vector<Cone> robotCones);
	double calcViewOfGoal(Vector2 testPosition, roboteam_msgs::World world);
	std::pair<std::vector<double>, std::vector<double>> getOpenGoalAngles(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistToSelf(Vector2 testPosition, roboteam_msgs::World world);
	double calcBallReflectionAngle(Vector2 testPosition, roboteam_msgs::World world);

	void setCloseToPos(Vector2 closeToPos);
	// boost::optional<double> computePassPointScore(Vector2 testPosition);
	double computeScore(Vector2 testPosition);
	double computeScore(Vector2 testPosition, roboteam_msgs::World world);
	Vector2 computeBestOpportunity(Vector2 centerPoint, double boxLength, double boxWidth);
	BestTeammate chooseBestTeammate(bool notBackwards=false, bool distFromTargetLimit=false, double limit=8.0, bool realWorld=false);
private:
	
	// Weights for determining the score of a point on the field, including min and max values that correspond to the min and max scores
	double distToGoalWeight;
	double distToGoalMin;
	double distToGoalMax;
	double viewOfGoalWeight;
	double viewOfGoalMin;
	double viewOfGoalMax;
	double distToOppWeight;
	double distToOppMin;
	double distToOppMax;
	double distToTeammateWeight;
	double distToTeammateMin;
	double distToTeammateMax;
	double distToBallWeight;
	double distToBallMin;
	double distToBallMax;
	double distToSelfWeight;
	double distToSelfMin;
	double distToSelfMax;
	double ballReflectionAngleWeight;
	double ballReflectionAngleMin;
	double ballReflectionAngleMax;
	double distOppToBallTrajWeight;
	double distOppToBallTrajMin;
	double distOppToBallTrajMax;
	double distOppToTargetTrajWeight;
	double distOppToTargetTrajMin;
	double distOppToTargetTrajMax;

	double totalWeight = 0.0001;

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
