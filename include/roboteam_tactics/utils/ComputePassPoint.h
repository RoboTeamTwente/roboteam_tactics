#pragma once

#include "ros/ros.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/Math.h"
#include "roboteam_tactics/utils/Draw.h"
#include "roboteam_tactics/utils/Cone.h"

#include <vector>
#include <string>

namespace rtt {

class PassPoint {
public:
	PassPoint();
	double calcDistToClosestOpp(roboteam_utils::Vector2 testPosition, roboteam_msgs::World world);
	double calcDistOppToBallTraj(roboteam_utils::Vector2 testPosition, roboteam_msgs::World world);
	std::vector<Cone> combineOverlappingRobots(std::vector<Cone> robotCones);
	double calcViewOfGoal(roboteam_utils::Vector2 testPosition, roboteam_msgs::World world);
	double computePassPointScore(roboteam_utils::Vector2 testPosition);
	double computeBestPassPoint();
private:
	double distToGoalWeight;
	double distToOppWeight;
	double distToBallWeight;
	double viewOfGoalWeight;
	double distOppToBallTrajWeight;

	Draw drawer;
};

} // rtt