#include "roboteam_msgs/World.h"

#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(CanReachPoint);

CanReachPoint::CanReachPoint(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

// roboteam_utils::Vector2 CanReachPoint::ComputeMaxAcceleration(double angle, roboteam_utils::Vector2 maxAcc) {
// 	double a = maxAcc.x;
// 	double b = maxAcc.y;
// 	angle = 0.5*M_PI - angle;
// 	double c = a - (a*a/(b * tan(angle) + a));
// 	double d = a / (tan(angle) + (a/b));
// 	roboteam_utils::Vector2 maxAccInDirection = roboteam_utils::Vector2(c, d);
// 	return maxAccInDirection;
// }

double CanReachPoint::estimateTimeToPoint(roboteam_utils::Vector2 currentPos, roboteam_utils::Vector2 currentVel, roboteam_utils::Vector2 targetPos) {
	roboteam_utils::Vector2 posDiff = targetPos - currentPos;

	roboteam_utils::Vector2 targetVel = posDiff.normalize().scale(posPGain);
	roboteam_utils::Vector2 velDiff = targetVel - currentVel;
	double timeToReachVel = velDiff.length() / maxAcc; 
	double distanceToReachVel = velDiff.length()/2 * timeToReachVel;

	double timeToStop = decelerationDistance / (targetVel.length()/2); // deceleration time = deceleration distance / average speed during deceleration

	if (posDiff.length() < (distanceToReachVel + decelerationDistance)) {
		ROS_INFO_STREAM("hmm, distance too short");
		return -1.0;
	} else {
		double distAtMaxVel = posDiff.length() - (distanceToReachVel + decelerationDistance);
		double timeAtMaxVel = distAtMaxVel / maxVel;
		return (timeToReachVel + timeToStop + timeAtMaxVel);
	}
}

bt::Node::Status CanReachPoint::Update() {
	roboteam_msgs::World world = LastWorld::get();
	int ROBOT_ID = GetInt("ROBOT_ID");
	double xGoal = GetDouble("xGoal");
	double yGoal = GetDouble("yGoal");

	// Wait for the first world message
	while (world.us.size() == 0) {
		return Status::Running;
	}

	roboteam_utils::Vector2 currentPos(world.us.at(ROBOT_ID).pos);
	roboteam_utils::Vector2 currentVel(world.us.at(ROBOT_ID).vel);
	roboteam_utils::Vector2 targetPos(xGoal, yGoal);

	double estimatedTimeToPoint = estimateTimeToPoint(currentPos, currentVel, targetPos);

	if (estimatedTimeToPoint < 0.0) {
		ROS_INFO_STREAM("hmm, distance too short");
		return Status::Failure;
	} else {
		double timeLimit = GetDouble("timeLimit");
		if (estimatedTimeToPoint < timeLimit) {
			return Status::Success;
		} else {
			return Status::Failure;
		}
	}
}

} // rtt 
