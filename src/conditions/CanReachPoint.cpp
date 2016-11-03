#include "roboteam_tactics/conditions/CanReachPoint.h"
#include "roboteam_tactics/utils/LastWorld.h"
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

double CanReachPoint::cleanAngle(double angle) {
	if (angle <= M_PI && angle >= -M_PI) {
		return angle;
	} else if (angle > M_PI) {
		angle -= 2*M_PI;
		return cleanAngle(angle);
	} else if (angle < -M_PI) {
		angle += 2*M_PI;
		return cleanAngle(angle);
	}
	ROS_WARN("yo, whatcha doin' here");
	return 0.0;
}

bt::Node::Status CanReachPoint::Update() {
	
	// Set max velocities etc.. 
	double time = 0.0;
	double speed = 0.0;
	double travelledDistance = 0.0;
	double timeStep = 0.01;
	double maxRotSpeed = 6.3;
	double maxSpeed = 2.0;
	roboteam_utils::Vector2 maxAcc = roboteam_utils::Vector2(3.5, 2.0);
	double maxDec = 5.0;


	// Get world and blackboard information
	roboteam_msgs::World world = LastWorld::get();
	int myID = GetInt("ROBOT_ID");
	bool shouldStop = GetBool("shouldStop");
	double xTarget = GetDouble("xGoal");
	double yTarget = GetDouble("yGoal");
	double angleGoal = GetDouble("angleGoal");
	double timeLimit = GetDouble("timeLimit");

	roboteam_utils::Vector2 myPos;
	double myAngle;
	if (GetString("whichTeam") == "us") {
		myPos = roboteam_utils::Vector2(world.us.at(myID).pos.x, world.us.at(myID).pos.y); 
		myAngle = world.us.at(myID).angle;
	} else if (GetString("whichTeam") == "them") {
		myPos = roboteam_utils::Vector2(world.them.at(myID).pos.x, world.them.at(myID).pos.y); 
		myAngle = world.them.at(myID).angle;
	} else {
		ROS_WARN("No team specified...");
	}
	roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(xTarget, yTarget);
	roboteam_utils::Vector2 differenceVector = targetPos - myPos;
	
	
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
		angleDiff = cleanAngle(angleDiff);
		roboteam_utils::Vector2 maxAccInDirection = ComputeMaxAcceleration(fabs(angleDiff), maxAcc);
		speed += maxAccInDirection.length()*timeStep;
		if (speed > maxSpeed) {speed = maxSpeed;}
		travelledDistance += speed*timeStep;
		
		// Rotate towards your goal, so you get maximum acceleration
		if (fabs(angleDiff) > maxRotSpeed*timeStep) {
			if (angleDiff > 0) {
 				myAngle += maxRotSpeed*timeStep;
			} else {
				myAngle -= maxRotSpeed*timeStep;
			}
		} else {
			myAngle = differenceVector.angle();
		}

		time += timeStep;
		if (time > 5.0) {break;} // Stop before it becomes an endless loop

		// How much time and distance would it take to rotate to our target angle?
		myAngle = cleanAngle(myAngle);
		double angleError = angleGoal - myAngle;
		angleError = cleanAngle(angleError);
		double rotationTime = angleError / maxRotSpeed;
		double rotationDistance = rotationTime * speed;

		if (shouldStop) { // If we should take into account deceleration time and distance
			double decelerationTime = speed / maxDec;
			double decelerationDistance = 0.5 * maxDec * decelerationTime*decelerationTime;
			if (travelledDistance + rotationDistance + decelerationDistance >= differenceVector.length()) {
				time += (decelerationTime + rotationTime);
				ROS_INFO_STREAM("time: " << time << " distance " << travelledDistance + rotationDistance + decelerationDistance);
				break;
			}
		} else {
			if (travelledDistance + rotationDistance >= differenceVector.length()) {
				time += rotationTime;
				ROS_INFO_STREAM("time: " << time << " distance " << travelledDistance+rotationDistance);
				break;
			}
		}
	}
	double timeWithRotation = time;


	// See how many iterations it takes without rotation towards the goal, this may be faster when
	// the robot is already close to the goal
	if (GetString("whichTeam") == "us") {
		myPos = roboteam_utils::Vector2(world.us.at(myID).pos.x, world.us.at(myID).pos.y); 
		myAngle = world.us.at(myID).angle;
	} else if (GetString("whichTeam") == "them") {
		myPos = roboteam_utils::Vector2(world.them.at(myID).pos.x, world.them.at(myID).pos.y); 
		myAngle = world.them.at(myID).angle;
	} else {
		ROS_WARN("No team specified...");
	}
	time = 0.0;
	speed = 0.0;
	travelledDistance = 0.0;
	while (travelledDistance < differenceVector.length()) {
		double angleDiff = differenceVector.angle() - myAngle;
		angleDiff = cleanAngle(angleDiff);
		// ROS_INFO_STREAM("angleDiff: " << angleDiff);
		roboteam_utils::Vector2 maxAccInDirection = ComputeMaxAcceleration(fabs(angleDiff), maxAcc);
		// ROS_INFO_STREAM("maxAcc: " << maxAccInDirection.length());
		speed += maxAccInDirection.length()*timeStep;
		if (speed > maxSpeed) {speed = maxSpeed;}
		travelledDistance += speed*timeStep;
		
		time += timeStep;
		if (time > 5.0) {break;} // Stop before it becomes an endless loop

		myAngle = cleanAngle(myAngle);
		double angleError = angleGoal - myAngle;
		angleError = cleanAngle(angleError);
		double rotationTime = angleError / maxRotSpeed;
		// At current speed, how much distance would this take?
		double rotationDistance = rotationTime * speed;

		if (shouldStop) { // If we should take into account deceleration time and distance
			double decelerationTime = speed / maxDec;
			double decelerationDistance = 0.5 * maxDec * decelerationTime*decelerationTime;
			if (travelledDistance + rotationDistance + decelerationDistance >= differenceVector.length()) {
				time += (decelerationTime + rotationTime);
				ROS_INFO_STREAM("time: " << time << " distance " << travelledDistance + rotationDistance + decelerationDistance);
				break;
			}
		} else {
			if (travelledDistance + rotationDistance >= differenceVector.length()) {
				time += rotationTime;
				ROS_INFO_STREAM("time: " << time << " distance " << travelledDistance+rotationDistance);
				break;
			}
		}
	}

	// If this time without rotation is larger, use the time with rotation
	if (time > timeWithRotation) {
		time = timeWithRotation;
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
