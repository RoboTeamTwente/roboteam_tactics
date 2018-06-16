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
	float score;
};

class OpportunityFinder {

public:
	OpportunityFinder();
	void Initialize(std::string fileName, int ROBOT_ID, std::string target, int targetID);

	void setWeight(std::string metric, double value);
	void setMin(std::string metric, double value);
	void setMax(std::string metric, double value);

	double calcDistToClosestOpp(const Vector2& testPosition, const roboteam_msgs::World& world);
	double calcDistToClosestTeammate(const Vector2& testPosition, const roboteam_msgs::World& world);
	double calcAngleToClosestTeammate(const Vector2& testPosition, const roboteam_msgs::World& world);
	double calcDistOppToBallTraj(const Vector2& testPosition, const roboteam_msgs::World& world, double chipMargin=0.0);
	double calcDistOppToTargetTraj(const Vector2& testPosition, const roboteam_msgs::World& world);
	std::vector<Cone> combineOverlappingRobots(std::vector<Cone> robotCones);
	double calcViewOfGoal(const Vector2& testPosition, const roboteam_msgs::World& world);
	std::pair<double, double> calcBestViewOfGoal(const Vector2& testPosition, const roboteam_msgs::World& world);
	std::pair<std::vector<double>, std::vector<double>> getOpenGoalAngles(const Vector2& testPosition, const roboteam_msgs::World& world);
	double calcDistToSelf(const Vector2& testPosition, const roboteam_msgs::World& world);
	double calcBallReflectionAngle(const Vector2& testPosition, const roboteam_msgs::World& world);

	void setCloseToPos(Vector2 closeToPos);
	double computeScore(const Vector2& testPosition);
	double computeScore(const Vector2& testPosition, const roboteam_msgs::World& world);
	Vector2 computeBestOpportunity(Vector2 centerPoint, double boxLength, double boxWidth);
	BestTeammate chooseBestTeammate(bool realScore=false, bool realPos=false, bool notBackwards=false, bool distFromTargetLimit=false, double limit=8.0);
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
	double angleToTeammateWeight;
	double angleToTeammateMin;
	double angleToTeammateMax;
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
	bool goalIsCrossArea;

	bool isCloseToPosSet;
	Vector2 closeToPos;
	
	roboteam_msgs::GeometryFieldSize field;

	Draw drawer;
	std::vector<std::string> names;
};

} // rtt
