#pragma once

#include "ros/ros.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_utils/Cone.h"

#include <vector>
#include <string>

namespace rtt {

class PassPoint {
public:
	PassPoint();
	double calcDistToClosestOpp(Vector2 testPosition, roboteam_msgs::World world);
	double calcDistOppToBallTraj(Vector2 testPosition, roboteam_msgs::World world);
	std::vector<Cone> combineOverlappingRobots(std::vector<Cone> robotCones);
	double calcViewOfGoal(Vector2 testPosition, roboteam_msgs::World world);
	double computePassPointScore(Vector2 testPosition);
	Vector2 computeBestPassPoint();
private:
	double distToGoalWeight;
	double distToOppWeight;
	double distToBallWeight;
	double viewOfGoalWeight;
	double distOppToBallTrajWeight;

	Draw drawer;
};

} // rtt