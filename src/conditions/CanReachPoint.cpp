#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

CanReachPoint::CanReachPoint(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    
}

roboteam_utils::Vector2 CanReachPoint::ComputeMaxAcceleration(double angle, roboteam_utils::Vector2 maxAcc) {
	double a = maxAcc.x;
	double b = maxAcc.y;
	angle = 0.5*M_PI - angle;
	double c = a - (a*a/(b * tan(angle) + a));
	double d = a / (tan(angle) + (a/b));
	roboteam_utils::Vector2 maxAccInDirection = roboteam_utils::Vector2(c, d);
	return maxAccInDirection;
}

bt::Node::Status CanReachPoint::Update() {
	
	// Set max velocities etc.. 
	double time = 0.0;0
	double speed = 0.0;
	double travelledDistance = 0.0;
	double timeStep = 0.01;
	double maxRotSpeed = 6.3;
	double maxSpeed = 2.0;
	roboteam_utils::Vector2 maxAcc = roboteam_utils::Vector2(3.5, 2.0);


	// Get world information
	roboteam_msgs::World world = LastWorld::get();
	int myID = GetInt("ROBOT_ID");
	double xTarget = GetDouble("xGoal");
	double yTarget = GetDouble("yGoal");
	double timeLimit = GetDouble("timeLimit");
	
	roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us.at(myID).pos.x, world.us.at(myID).pos.y); 
	roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(xTarget, yTarget);
	roboteam_utils::Vector2 differenceVector = targetPos - myPos;
	double myAngle = world.us.at(myID).angle;
	
	
	// Calculate our orientation with respect to the direction we have to move
	double angleDiff = differenceVector.angle() - myAngle;
	if (angleDiff < M_PI) {angleDiff += 2*M_PI;}
	if (angleDiff > M_PI) {angleDiff -= 2*M_PI;}

	if (fabs(angleDiff) >= 0.5*M_PI) {  // In this case driving backwards is faster
		if (myAngle > 0) {myAngle -= M_PI;}
		if (myAngle <= 0) {myAngle += M_PI;}
	}


	// See how many iterations it takes before the required distance is travelled with the maximum acceleration
	while (travelledDistance < differenceVector.length()) {
		double angleDiff = differenceVector.angle() - myAngle;
		if (angleDiff < M_PI) {angleDiff += 2*M_PI;}
		if (angleDiff > M_PI) {angleDiff -= 2*M_PI;}
		
		roboteam_utils::Vector2 maxAccInDirection = ComputeMaxAcceleration(fabs(angleDiff), maxAcc);
		speed += maxAccInDirection.length()*timeStep;
		if (speed > maxSpeed) {speed = maxSpeed;}
		travelledDistance += speed*timeStep;
		
		if (fabs(angleDiff) > maxRotSpeed*timeStep) {
			if (angleDiff > 0) {
 				myAngle += maxRotSpeed*timeStep;
			} else {
				myAngle -= maxRotSpeed*timeStep;
			}
		} else {
			myAngle = differenceVector.angle();
		}
		// ROS_INFO_STREAM("angleDiff: " << angleDiff << " maxAcc: " << maxAccInDirection.length() << " travelledDistance: " << travelledDistance);
		time += timeStep;
		if (time > 5.0) {break;}
	}


	// If the calculated time is less than the given limit, return succes
	if (time < timeLimit) {
		return Status::Success;
	} else {
		return Status::Failure;
	}
	return Status::Invalid;
}

} // rtt 