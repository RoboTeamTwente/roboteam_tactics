#include "roboteam_msgs/World.h"

#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

RTT_REGISTER_CONDITION(CanReachPoint);

CanReachPoint::CanReachPoint(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

// Vector2 CanReachPoint::ComputeMaxAcceleration(double angle, Vector2 maxAcc) {
// 	double a = maxAcc.x;
// 	double b = maxAcc.y;
// 	angle = 0.5*M_PI - angle;
// 	double c = a - (a*a/(b * tan(angle) + a));
// 	double d = a / (tan(angle) + (a/b));
// 	Vector2 maxAccInDirection = Vector2(c, d);
// 	return maxAccInDirection;
// }

double CanReachPoint::estimateTimeToPoint(Vector2 currentPos, Vector2 currentVel, Vector2 targetPos) {
	Vector2 posDiff = targetPos - currentPos;

	/*
	Vector2 targetVel = posDiff.normalize().scale(posPGain);
	Vector2 velDiff = targetVel - currentVel;
	double timeToReachVel = velDiff.length() / maxAcc; 
	double distanceToReachVel = velDiff.length()/2 * timeToReachVel;

	double timeToStop = decelerationDistance / (targetVel.length()/2); // deceleration time = deceleration distance / average speed during deceleration

	//ROS_INFO_STREAM("distanceToReachVel: " << distanceToReachVel << " decelerationDistance: " << decelerationDistance);

	if (posDiff.length() < (distanceToReachVel + decelerationDistance)) {
		return -1.0;
	} else {
		double distAtMaxVel = posDiff.length() - (distanceToReachVel + decelerationDistance);
		double timeAtMaxVel = distAtMaxVel / maxVel;
		return (timeToReachVel + timeToStop + timeAtMaxVel);
	}
	*/

	// simple version because acceleration and decaleration calculation where not complete yet
	double averageVel=3.0; // m/s
	ROS_INFO_STREAM("currentPos x:"<<currentPos.x<<" y:"<<currentPos.y<<"; targetPos x"<<targetPos.x<<" y:"<<targetPos.y<<" length:"<<posDiff.length());
	return posDiff.length()/averageVel;
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

	Vector2 currentPos(getWorldBot(ROBOT_ID)->pos);
	Vector2 currentVel(getWorldBot(ROBOT_ID)->vel);
	Vector2 targetPos(xGoal, yGoal);

	if(HasString("whichTeam") && GetString("whichTeam")=="them"){
		currentPos=world.them.at(ROBOT_ID).pos;
		currentVel=world.them.at(ROBOT_ID).vel;
		ROS_INFO_STREAM("using their team");
	}

	

	double estimatedTimeToPoint = estimateTimeToPoint(currentPos, currentVel, targetPos);

	ROS_INFO_STREAM("estimatedTimeToPoint: " << estimatedTimeToPoint);

	if (estimatedTimeToPoint < 0.0) {
		// ROS_INFO_STREAM("hmm, distance too short");
		return Status::Failure;
	} else {
		double timeLimit = GetDouble("timeLimit");
		if (estimatedTimeToPoint < timeLimit) {
			// ROS_INFO_STREAM("can reach point within time");
			return Status::Success;
		} else {
			// ROS_INFO_STREAM("cannot reach point within time");
			return Status::Failure;
		}
	}
}

} // rtt 
